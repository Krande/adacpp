// Envelopes: element-wise max / min of one field over a list of cases, with the governing case.
//
// Fed one stride per case, in case order (case index = position in that order, 0-based):
//
//   max[i] = the largest non-NaN value seen at i; gov_max[i] = the case it came from
//   min[i] = the smallest non-NaN value seen at i; gov_min[i] = the case it came from
//
// Ties keep the EARLIER case (strict comparison) -- np.argmax / np.nanargmax semantics. NaN is
// skipped (np.fmax / np.nanargmax semantics); a position that is NaN in every case stays NaN with
// governing case 0. Comparisons only, no arithmetic: the result is exact on every engine.
//
// Governing indices are uint16: at most 65535 cases per envelope.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace adacpp::fea {

class EnvelopeAccumulator {
public:
    explicit EnvelopeAccumulator(std::size_t n);

    // Fold in the next case's stride (n floats). Case index = number of strides added before it.
    void add(const float *x);

    std::size_t size() const {
        return max_.size();
    }
    std::size_t cases() const {
        return cases_;
    }
    const std::vector<float> &max() const {
        return max_;
    }
    const std::vector<float> &min() const {
        return min_;
    }
    const std::vector<uint16_t> &gov_max() const {
        return gov_max_;
    }
    const std::vector<uint16_t> &gov_min() const {
        return gov_min_;
    }

private:
    std::vector<float> max_, min_;
    std::vector<uint16_t> gov_max_, gov_min_;
    std::size_t cases_ = 0;
};

// One-shot over in-memory strides.
void envelope_strides(const std::vector<const float *> &in, std::size_t n, float *out_max, float *out_min,
                      uint16_t *gov_max, uint16_t *gov_min);

} // namespace adacpp::fea
