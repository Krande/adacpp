//
// Created by ofskrand on 23.01.2024.
//
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#ifndef STEP_TO_GLB_H
#define STEP_TO_GLB_H

#include <tiny_gltf.h>
#include <string>
#include <TopoDS_Shape.hxx>


// Forward declaration of ProcessShape function
void ProcessShape(const TopoDS_Shape& shape, tinygltf::Model& gltfModel);

// Function to convert STEP file to GLB file
void stp_to_glb(const std::string& stp_file, const std::string& glb_file);


#endif //STEP_TO_GLB_H
