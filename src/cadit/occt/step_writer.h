#ifndef NANO_OCCT_STEP_WRITER_H
#define NANO_OCCT_STEP_WRITER_H

#include <filesystem>
#include <STEPCAFControl_Writer.hxx>

void write_boxes_to_step(const std::string &filename, const std::vector<std::vector<float>> &box_origins,
                         const std::vector<std::vector<float>> &box_dims);

void write_box_to_step(const std::string &filename, const std::vector<float> &box_origin,
                       const std::vector<float> &box_dims);

std::string step_writer_to_string(STEPCAFControl_Writer& writer);

#endif // NANO_OCCT_STEP_WRITER_H