#ifndef NANO_OCCT_STEP_WRITER_H
#define NANO_OCCT_STEP_WRITER_H

#include <filesystem>
#include <string>
#include <vector>
#include <STEPCAFControl_Writer.hxx>
#include <TopoDS_Shape.hxx>
#include "../../geom/Color.h"

void write_boxes_to_step(const std::string &filename, const std::vector<std::vector<float>> &box_origins,
                         const std::vector<std::vector<float>> &box_dims);

void write_box_to_step(const std::string &filename, const std::vector<float> &box_origin,
                       const std::vector<float> &box_dims);

std::string step_writer_to_string(STEPCAFControl_Writer &writer);

// Write arbitrary shapes (with per-shape name + color) to a STEP file via the
// OCAF/XCAF document model. Backs adapy's StepWriter under the adacpp backend.
void write_shapes_to_step(const std::string &filename, const std::vector<TopoDS_Shape> &shapes,
                          const std::vector<std::string> &names, const std::vector<Color> &colors,
                          const std::string &unit, const std::string &schema, const std::string &top_level_name);

#endif // NANO_OCCT_STEP_WRITER_H