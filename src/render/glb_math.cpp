#include "glb_math.h"

#include <algorithm>
#include <cmath>

namespace adacpp::render::detail {

Mat4 mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += a[k * 4 + row] * b[c * 4 + k];
            r[c * 4 + row] = s;
        }
    return r;
}

Mat4 node_matrix(const tinygltf::Node &n) {
    Mat4 m = kIdentity;
    if (n.matrix.size() == 16) {
        for (int i = 0; i < 16; ++i)
            m[i] = n.matrix[i];
        return m;
    }
    double t[3] = {0, 0, 0}, s[3] = {1, 1, 1}, q[4] = {0, 0, 0, 1};
    for (size_t i = 0; i < n.translation.size() && i < 3; ++i)
        t[i] = n.translation[i];
    for (size_t i = 0; i < n.scale.size() && i < 3; ++i)
        s[i] = n.scale[i];
    for (size_t i = 0; i < n.rotation.size() && i < 4; ++i)
        q[i] = n.rotation[i];
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                         2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            m[c * 4 + row] = r[c * 3 + row] * s[c];
    m[12] = t[0];
    m[13] = t[1];
    m[14] = t[2];
    return m;
}

Mat4 compose(const double t[3], const float q[4], const float s[3]) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                         2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                         2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    Mat4 m = kIdentity;
    for (int c = 0; c < 3; ++c)
        for (int row = 0; row < 3; ++row)
            m[c * 4 + row] = r[c * 3 + row] * s[c];
    m[12] = t[0];
    m[13] = t[1];
    m[14] = t[2];
    return m;
}

double det3(const Mat4 &m) {
    return m[0] * (m[5] * m[10] - m[6] * m[9]) - m[4] * (m[1] * m[10] - m[2] * m[9]) +
           m[8] * (m[1] * m[6] - m[2] * m[5]);
}

std::array<float, 16> to_float(const Mat4 &m) {
    std::array<float, 16> f;
    for (int i = 0; i < 16; ++i)
        f[i] = float(m[i]);
    return f;
}

void normalize(float v[3]) {
    float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l > 0)
        for (int i = 0; i < 3; ++i)
            v[i] /= l;
}

} // namespace adacpp::render::detail
