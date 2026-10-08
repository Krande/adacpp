// Derivation ops: the NON-linear result components, re-derived from superposed linear components.
//
// A load combination superposes only linear components (superpose.h). Anything that is not linear in
// the load -- von Mises, principal stresses, vector magnitudes -- must be recomputed from the combined
// linear components, never superposed. Each op here is a port of adapy's numpy kernels
// (derived_values.py, and the displacement magnitude in derived_fields.py) with THE
// SAME OPERATION ORDER, so the result rounds the same way:
//
//   inputs  float32 -> promoted to double (np.asarray(x, dtype=float))
//   math    double, one IEEE op at a time, left-to-right as Python evaluates the expression
//   output  rounded to float32 once (the blob writer's np.asarray(values, dtype=float32))
//
//   plane_von_mises(sx, sy, txy)      sqrt(sx*sx + sy*sy - sx*sy + 3.0*txy*txy)
//   plane_principal(sx, sy, txy)      c = 0.5*(sx + sy); r = sqrt((0.5*(sx - sy))**2 + txy*txy)
//                                     -> (c + r, c - r)                        [P1, P2]
//   magnitude(x0, .., xk)  k < 7      sqrt(((x0*x0 + x1*x1) + x2*x2) ...)      np.linalg.norm order
//   shell_decompose(bottom[3], top[3]) membrane m = 0.5*(top + bottom); bending b = 0.5*(top - bottom)
//                                     -> (m0, m1, b0, b1, m2, b2, plane_von_mises(m0, m1, m2))
//                                        [SIGMX, SIGMY, SIGBX, SIGBY, TAUMXY, TAUBXY, MVONMISES]
//   shell_resultants(d0..d5, t)       t2 = t*t/6.0
//                                     -> (d0*t, d4*t, d1*t, d5*t2, d2*t2, d3*t2)
//                                        [NXX, NXY, NYY, MXY, MXX, MYY]
//   copy(x)                           x (passthrough column, for re-laid-out outputs)
//
// The membrane principal stresses (PM-STRESS) are plane_principal over (SIGMX, SIGMY, TAUMXY), and
// G-STRESS VONMISES is plane_von_mises over (SIGXX, SIGYY, TAUXY): the op is the same, only the
// argument columns differ, so they are not separate ops.
//
// `**2` is evaluated as h*h: numpy's float power special-cases the exponent 2 to a square.
//
// The single-output names of the lazy-case manifest (adapy artefacts/combine.py DERIVATION_OPS) are
// accepted too, so a manifest's `derived_components` entry maps 1:1 onto one op here:
//   magnitude3 = magnitude over 3 args; plane_principal_1 / plane_principal_2 = P1 / P2 alone.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace adacpp::fea {

enum class DeriveOp {
    Copy,
    PlaneVonMises,
    PlanePrincipal,
    Magnitude,
    ShellDecompose,
    ShellResultants,
    PlanePrincipal1, // P1 only
    PlanePrincipal2, // P2 only
    Magnitude3,      // magnitude, exactly 3 args
};

// "copy" | "plane_von_mises" | "plane_principal" | "plane_principal_1" | "plane_principal_2" |
// "magnitude" | "magnitude3" | "shell_decompose" | "shell_resultants". Also accepts the adapy aliases
// "general_stress" (= plane_von_mises), "membrane_principal" (= plane_principal), "decompose_shell"
// (= shell_decompose) and "stress_resultants" (= shell_resultants). Throws std::invalid_argument.
DeriveOp derive_op_from_name(const std::string &name);
const char *derive_op_name(DeriveOp op);

// Argument / output column counts. Magnitude takes 1..7 arguments (np.linalg.norm sums fewer than 8
// values left to right; beyond that numpy switches to pairwise summation, which this op does not
// claim to reproduce).
std::size_t derive_op_n_args(DeriveOp op, std::size_t requested_args);
std::size_t derive_op_n_out(DeriveOp op);

// One op applied over a stride: reads `args` columns of `in` (row width `in_ncomp`), writes `out`
// columns of `out` (row width `out_ncomp`). An `out` entry of -1 drops that output (e.g. keep only
// P1). `in` and `out` may be the SAME buffer (in-place on a field whose derived columns sit next to
// its linear ones): each row's arguments are read before any of its outputs is written.
//
// shell_resultants needs a per-row thickness: `thickness[r / thickness_rows]`, i.e. one thickness
// per `thickness_rows` consecutive rows (n_ips for an element field, 1 for a nodal one).
struct DeriveSpec {
    DeriveOp op = DeriveOp::Copy;
    std::vector<int> args;
    std::vector<int> out;
};

void apply_derive(const DeriveSpec &spec, const float *in, std::size_t in_ncomp, float *out, std::size_t out_ncomp,
                  std::size_t rows, const double *thickness = nullptr, std::size_t thickness_rows = 1);

// Validates column indices / counts against the row widths. Throws std::invalid_argument.
void validate_derive(const DeriveSpec &spec, std::size_t in_ncomp, std::size_t out_ncomp, bool have_thickness);

} // namespace adacpp::fea
