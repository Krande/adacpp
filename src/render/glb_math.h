// Transform maths of the GLB reader (internal): column-major 4x4 matrices and the boxes they carry. In
// double, so a node far from the origin, or GlbOptions::offset, costs no precision before a vertex or an
// instance matrix becomes a float.
#pragma once

#include <algorithm>
#include <array>

#include "glb_gltf.h"

namespace adacpp::render::detail {

using Mat4 = std::array<double, 16>;

inline const Mat4 kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

Mat4 mul(const Mat4 &a, const Mat4 &b);
// The node's local transform: its matrix, else its translation, rotation and scale.
Mat4 node_matrix(const tinygltf::Node &n);
Mat4 compose(const double t[3], const float q[4], const float s[3]);
double det3(const Mat4 &m);
// What the renderer takes.
std::array<float, 16> to_float(const Mat4 &m);
// `box` (min xyz, max xyz) under `m`, as the box around its eight corners, added to `out`.
template <typename T> void grow_bounds(T out[6], const Mat4 &m, const T box[6]) {
    for (int corner = 0; corner < 8; ++corner) {
        const double p[3] = {box[(corner & 1) ? 3 : 0], box[(corner & 2) ? 4 : 1], box[(corner & 4) ? 5 : 2]};
        for (int k = 0; k < 3; ++k) {
            const T w = T(m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k]);
            out[k] = std::min(out[k], w);
            out[3 + k] = std::max(out[3 + k], w);
        }
    }
}
void normalize(float v[3]);

} // namespace adacpp::render::detail
