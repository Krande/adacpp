#include "superpose.h"

#include <algorithm>
#include <stdexcept>

namespace adacpp::fea {

void scale_into(float *out, const float *x, float c, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i)
        out[i] = c * x[i];
}

void accumulate(float *__restrict out, const float *__restrict x, float c, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        const float p = c * x[i]; // rounded to float32 on its own -- never fused into the add
        out[i] = out[i] + p;
    }
}

void combine_strides(std::span<const float *const> in, std::span<const float> c, float *out, std::size_t n) {
    if (in.empty() || in.size() != c.size())
        throw std::invalid_argument("combine_strides: need >= 1 term and one factor per input");
    // 16 K floats = 64 KB of output per block: stays in L2 while every term streams past it.
    constexpr std::size_t BLOCK = 16384;
    for (std::size_t b = 0; b < n; b += BLOCK) {
        const std::size_t m = std::min(BLOCK, n - b);
        scale_into(out + b, in[0] + b, c[0], m);
        for (std::size_t t = 1; t < in.size(); ++t)
            accumulate(out + b, in[t] + b, c[t], m);
    }
}

} // namespace adacpp::fea
