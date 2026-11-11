// Transform maths of the GLB reader (internal): column-major 4x4 matrices and the boxes they carry.
#pragma once

#include <array>

#include "glb_gltf.h"

namespace adacpp::render::detail {

using Mat4 = std::array<float, 16>;

inline const Mat4 kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

Mat4 mul(const Mat4 &a, const Mat4 &b);
// The node's local transform: its matrix, else its translation, rotation and scale.
Mat4 node_matrix(const tinygltf::Node &n);
Mat4 compose(const float t[3], const float q[4], const float s[3]);
float det3(const Mat4 &m);
// `box` (min xyz, max xyz) under `m`, as the box around its eight corners, added to `out`.
void grow_bounds(float out[6], const Mat4 &m, const float box[6]);
void normalize(float v[3]);

} // namespace adacpp::render::detail
