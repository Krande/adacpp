#include <iostream>

#include "../../geom/Mesh.h"
#include "../../binding_core.h"
#include "../../visit/tess_helpers.h"
#include "../../helpers/helpers.h"
#include "tiny_gltf.h"
#include "tiny_helpers.h"
#include "tiny.h"




int write_to_gltf(const std::string& filename, const Mesh& mesh) {
    tinygltf::Model model;

    // Create a scene
    tinygltf::Scene scene;
    model.scenes.push_back(scene);
    model.defaultScene = 0;
    AddMesh(model, "mesh", mesh);

    // Save to file
    if (tinygltf::TinyGLTF gltf; !gltf.WriteGltfSceneToFile(&model, filename, true, true, true, false)) {
        std::cerr << "Failed to write glTF file" << std::endl;
        return -1;
    }

    return 0;
}

// take in a list of box dimensions and origins and write to step file using the AdaCPPStepWriter class
int write_boxes_to_gltf(const std::string &filename, const std::vector<std::vector<float>> &box_origins,
                         const std::vector<std::vector<float>> &box_dims) {
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


void tiny_gltf_module(nb::module_ &m) {
    m.def("write_mesh_to_gltf", &write_to_gltf, "filename"_a, "mesh"_a, "Write a Mesh to GLTF");
    m.def("write_boxes_to_gltf", &write_boxes_to_gltf, "filename"_a, "box_origins"_a, "box_dims"_a, "Write a list of boxes to GLTF");
}