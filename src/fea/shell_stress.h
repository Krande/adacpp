// Derived shell stresses as adapy's derived_values.py computes them: float64 out, widened from float32 or
// float64 in, every operation in numpy's order (left to right, x**2 as x*x), no FMA. So each result is the
// same bits as the numpy expression's.
//
// Arrays are row-major with the components last: `basic` / `bottom` / `top` are (rows, 3) as (SIGXX, SIGYY,
// TAUXY), `d` is (rows, 7) D-STRESS. `rows` counts everything before the component axis.
#pragma once

#include <cmath>
#include <cstddef>

namespace adacpp::fea::shell {

// sqrt(x*x + y*y - x*y + 3.0*t*t)
inline double plane_von_mises(double x, double y, double t) {
    return std::sqrt(x * x + y * y - x * y + 3.0 * t * t);
}

// (SIGXX, SIGYY, TAUXY, VONMISES): out (rows, 4).
template <class T> void general_stress(const T *basic, std::size_t rows, double *out);

// (P1, P2) of (x, y, t) read at columns cx, cy, ct of a (rows, ncols) table: out (rows, 2).
template <class T>
void plane_principal(const T *table, std::size_t rows, std::size_t ncols, std::size_t cx, std::size_t cy,
                     std::size_t ct, double *out);

// D-STRESS (SIGMX, SIGMY, SIGBX, SIGBY, TAUMXY, TAUBXY, MVONMISES): out (rows, 7).
template <class T> void decompose_shell(const T *bottom, const T *top, std::size_t rows, double *out);

// (NXX, NXY, NYY, MXY, MXX, MYY) from D-STRESS, one thickness per `per` rows: out (rows, 6).
void stress_resultants(const double *d, std::size_t rows, const double *thickness, std::size_t per, double *out);

} // namespace adacpp::fea::shell
