// Tier A superposition: a load combination as a float32 linear combination of stored-case strides.
//
// THE ORDER IS THE CONTRACT. For every element i:
//
//     out[i] = c[0] * x0[i]
//     out[i] = out[i] + c[1] * x1[i]
//     ...                                   (terms in the order given -- the file's term order)
//
// Each product and each sum is one float32 IEEE-754 operation (round to nearest even), so the result
// is bit-identical to numpy's `out = c1 * x1; out += c2 * x2; ...` on float32 arrays with float32
// factors, and identical on every engine: the kernels are compiled with -ffp-contract=off, so no
// compiler may fuse `out + c * x` into an FMA (which would round once instead of twice). How the
// loop is blocked or vectorised never changes a result, because no element depends on another.
//
// Factors arrive already rounded to float32 (FACT*cos(phi) formed in double and rounded once by the
// caller, as the solver-side reader does); this layer never sees a double factor.
#pragma once

#include <cstddef>
#include <span>

namespace adacpp::fea {

// out[i] = c * x[i]. `out` may alias `x` (scales in place).
void scale_into(float *out, const float *x, float c, std::size_t n);

// out[i] = out[i] + c * x[i]. `out` must not alias `x`.
void accumulate(float *out, const float *x, float c, std::size_t n);

// The whole combination over in-memory strides: `in.size() == c.size() >= 1`, each in[t] holding `n`
// floats. Blocked so every term streams through cache once per block; per-element order is exactly
// the one documented above. `out` must not alias any input.
void combine_strides(std::span<const float *const> in, std::span<const float> c, float *out, std::size_t n);

} // namespace adacpp::fea
