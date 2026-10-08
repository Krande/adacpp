#include "fea_arrays.h"

#include <charconv>
#include <cmath>
#include <limits>

namespace adacpp::fea {

StepStats compute_step_stats(const float *values, std::size_t rows, std::size_t n_components) {
    StepStats s;
    s.rows = rows;
    s.n_components = n_components;
    s.comp_min.assign(n_components, std::numeric_limits<float>::infinity());
    s.comp_max.assign(n_components, -std::numeric_limits<float>::infinity());
    s.comp_has.assign(n_components, 0);
    s.has_magnitude = n_components >= 3;
    float mag_min = std::numeric_limits<float>::infinity();
    float mag_max = -std::numeric_limits<float>::infinity();
    bool mag_any = false;

    for (std::size_t r = 0; r < rows; ++r) {
        const float *row = values + r * n_components;
        for (std::size_t c = 0; c < n_components; ++c) {
            const float v = row[c];
            if (!std::isfinite(v))
                continue;
            if (v < s.comp_min[c])
                s.comp_min[c] = v;
            if (v > s.comp_max[c])
                s.comp_max[c] = v;
            s.comp_has[c] = 1;
        }
        if (s.has_magnitude) {
            // np.linalg.norm(float32[..., :3], axis=-1): squares in float32, summed left to right
            // ((x0^2 + x1^2) + x2^2), then a float32 sqrt.
            const float a = row[0] * row[0];
            const float b = row[1] * row[1];
            const float c = row[2] * row[2];
            const float m = std::sqrt((a + b) + c);
            if (std::isfinite(m)) {
                if (m < mag_min)
                    mag_min = m;
                if (m > mag_max)
                    mag_max = m;
                mag_any = true;
            }
        }
    }
    for (std::size_t c = 0; c < n_components; ++c) {
        if (!s.comp_has[c]) {
            s.comp_min[c] = 0.0f;
            s.comp_max[c] = 0.0f;
        }
    }
    s.magnitude_finite = mag_any;
    s.mag_min = mag_any ? mag_min : 0.0f;
    s.mag_max = mag_any ? mag_max : 0.0f;
    return s;
}

void merge_step_stats(StepStats &acc, const StepStats &o) {
    if (acc.n_components == 0) {
        acc = o;
        return;
    }
    for (std::size_t c = 0; c < acc.n_components && c < o.n_components; ++c) {
        if (!o.comp_has[c])
            continue;
        if (!acc.comp_has[c]) {
            acc.comp_min[c] = o.comp_min[c];
            acc.comp_max[c] = o.comp_max[c];
            acc.comp_has[c] = 1;
            continue;
        }
        if (o.comp_min[c] < acc.comp_min[c])
            acc.comp_min[c] = o.comp_min[c];
        if (o.comp_max[c] > acc.comp_max[c])
            acc.comp_max[c] = o.comp_max[c];
    }
    if (o.magnitude_finite) {
        if (!acc.magnitude_finite) {
            acc.mag_min = o.mag_min;
            acc.mag_max = o.mag_max;
            acc.magnitude_finite = true;
        } else {
            if (o.mag_min < acc.mag_min)
                acc.mag_min = o.mag_min;
            if (o.mag_max > acc.mag_max)
                acc.mag_max = o.mag_max;
        }
    }
    acc.rows += o.rows;
}

std::string format_double(double v) {
    if (!std::isfinite(v))
        return "null";
    char buf[64];
    auto res = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, res.ptr);
}

std::string step_stats_json(const StepStats &s) {
    std::string out = "{\"rows\":" + std::to_string(s.rows) + ",\"n_components\":" + std::to_string(s.n_components) +
                      ",\"scalar_range_per_component\":[";
    for (std::size_t c = 0; c < s.n_components; ++c) {
        if (c)
            out += ',';
        out += '[' + format_double(s.comp_min[c]) + ',' + format_double(s.comp_max[c]) + ']';
    }
    // (0, 0) when there is no magnitude (fewer than 3 components, or nothing finite), as adapy's
    // writers record it.
    out += "],\"scalar_range_magnitude\":[" + format_double(s.mag_min) + ',' + format_double(s.mag_max) + "]}";
    return out;
}

} // namespace adacpp::fea
