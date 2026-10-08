#include "envelope.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace adacpp::fea {

EnvelopeAccumulator::EnvelopeAccumulator(std::size_t n) : max_(n), min_(n), gov_max_(n, 0), gov_min_(n, 0) {}

void EnvelopeAccumulator::add(const float *x) {
    if (cases_ >= 65536)
        throw std::length_error("envelope: more than 65536 cases (governing index is uint16)");
    const std::size_t n = max_.size();
    if (cases_ == 0) {
        std::copy(x, x + n, max_.begin());
        std::copy(x, x + n, min_.begin());
        ++cases_;
        return;
    }
    const uint16_t k = static_cast<uint16_t>(cases_);
    float *mx = max_.data();
    float *mn = min_.data();
    uint16_t *gx = gov_max_.data();
    uint16_t *gn = gov_min_.data();
    for (std::size_t i = 0; i < n; ++i) {
        const float v = x[i];
        // `cur != cur` is NaN: the first non-NaN value replaces a NaN, a NaN never replaces anything.
        const bool up = v > mx[i] || (mx[i] != mx[i] && v == v);
        const bool down = v < mn[i] || (mn[i] != mn[i] && v == v);
        mx[i] = up ? v : mx[i];
        gx[i] = up ? k : gx[i];
        mn[i] = down ? v : mn[i];
        gn[i] = down ? k : gn[i];
    }
    ++cases_;
}

void envelope_strides(const std::vector<const float *> &in, std::size_t n, float *out_max, float *out_min,
                      uint16_t *gov_max, uint16_t *gov_min) {
    if (in.empty())
        throw std::invalid_argument("envelope: need at least one case");
    EnvelopeAccumulator acc(n);
    for (const float *x : in)
        acc.add(x);
    std::copy(acc.max().begin(), acc.max().end(), out_max);
    std::copy(acc.min().begin(), acc.min().end(), out_min);
    std::copy(acc.gov_max().begin(), acc.gov_max().end(), gov_max);
    std::copy(acc.gov_min().begin(), acc.gov_min().end(), gov_min);
}

} // namespace adacpp::fea
