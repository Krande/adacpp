#include <cmath>

#include "shell_stress.h"

namespace adacpp::fea::shell {

template <class T> void general_stress(const T *basic, std::size_t rows, double *out) {
    for (std::size_t i = 0; i < rows; ++i) {
        const double x = basic[3 * i], y = basic[3 * i + 1], t = basic[3 * i + 2];
        double *o = out + 4 * i;
        o[0] = x;
        o[1] = y;
        o[2] = t;
        o[3] = plane_von_mises(x, y, t);
    }
}

template <class T>
void plane_principal(const T *table, std::size_t rows, std::size_t ncols, std::size_t cx, std::size_t cy,
                     std::size_t ct, double *out) {
    for (std::size_t i = 0; i < rows; ++i) {
        const T *r = table + i * ncols;
        const double x = r[cx], y = r[cy], t = r[ct];
        const double centre = 0.5 * (x + y);
        const double half = 0.5 * (x - y);
        const double radius = std::sqrt(half * half + t * t);
        out[2 * i] = centre + radius;
        out[2 * i + 1] = centre - radius;
    }
}

template <class T> void decompose_shell(const T *bottom, const T *top, std::size_t rows, double *out) {
    for (std::size_t i = 0; i < rows; ++i) {
        double m[3], b[3];
        for (int c = 0; c < 3; ++c) {
            const double lo = bottom[3 * i + c], hi = top[3 * i + c];
            m[c] = 0.5 * (hi + lo);
            b[c] = 0.5 * (hi - lo);
        }
        double *o = out + 7 * i;
        o[0] = m[0];
        o[1] = m[1];
        o[2] = b[0];
        o[3] = b[1];
        o[4] = m[2];
        o[5] = b[2];
        o[6] = plane_von_mises(m[0], m[1], m[2]);
    }
}

void stress_resultants(const double *d, std::size_t rows, const double *thickness, std::size_t per, double *out) {
    for (std::size_t i = 0; i < rows; ++i) {
        const double t = thickness[i / per];
        const double t2_over_6 = t * t / 6.0;
        const double *r = d + 7 * i;
        double *o = out + 6 * i;
        o[0] = r[0] * t;
        o[1] = r[4] * t;
        o[2] = r[1] * t;
        o[3] = r[5] * t2_over_6;
        o[4] = r[2] * t2_over_6;
        o[5] = r[3] * t2_over_6;
    }
}

template void general_stress<float>(const float *, std::size_t, double *);
template void general_stress<double>(const double *, std::size_t, double *);
template void plane_principal<float>(const float *, std::size_t, std::size_t, std::size_t, std::size_t, std::size_t,
                                     double *);
template void plane_principal<double>(const double *, std::size_t, std::size_t, std::size_t, std::size_t, std::size_t,
                                      double *);
template void decompose_shell<float>(const float *, const float *, std::size_t, double *);
template void decompose_shell<double>(const double *, const double *, std::size_t, double *);

} // namespace adacpp::fea::shell
