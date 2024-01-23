//
// Created by ofskrand on 23.01.2024.
//
#ifndef STEP_TO_GLB_H
#define STEP_TO_GLB_H

#include <string>
#include <TopoDS_Shape.hxx>


// Forward declaration of ProcessShape function

// Function to convert STEP file to GLB file
void stp_to_glb(const std::string& stp_file, const std::string& glb_file);


#endif //STEP_TO_GLB_H
