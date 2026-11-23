#include "effects.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace adacpp::render {

EmitterDesc smoke_emitter(const std::array<float, 3> &origin) {
    EmitterDesc e;
    e.origin = origin;
    e.direction = {0, 0, 1};
    e.spread = 0.35f;
    e.speed = 0.25f;
    e.speed_jitter = 0.5f;
    e.rate = 12.0f;
    e.lifetime = 4.0f;
    e.lifetime_jitter = 0.3f;
    e.gravity = {0, 0, 0.05f};
    e.drag = 0.4f;
    e.radius = 0.02f;
    e.size_start = 0.05f;
    e.size_end = 0.5f;
    e.color_start = {0.35f, 0.35f, 0.35f};
    e.intensity_start = 1.0f;
    e.alpha_start = 0.25f;
    e.color_end = {0.5f, 0.5f, 0.5f};
    e.intensity_end = 1.0f;
    e.alpha_end = 0.0f;
    e.stretch = 0.0f;
    e.blend = ParticleBlend::Alpha;
    return e;
}

namespace fx {

namespace {

constexpr float kPi = 3.14159265358979f;

uint32_t hash32(uint32_t x) {
    // lowbias32 (Chris Wellons)
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

// Smooth value noise in [-1, 1] at `x`, one random value per integer, smoothstep between them.
float value_noise(uint32_t seed, double x) {
    const double fl = std::floor(x);
    const auto i = int64_t(fl);
    const float f = float(x - fl);
    auto at = [&](int64_t k) {
        const uint32_t h = hash32(seed * 0x9E3779B9u ^ hash32(uint32_t(k) ^ hash32(uint32_t(uint64_t(k) >> 32))));
        return float(h) / 4294967295.0f * 2.0f - 1.0f;
    };
    const float s = f * f * (3.0f - 2.0f * f);
    return at(i) + (at(i + 1) - at(i)) * s;
}

float clamp01(float v) {
    return std::min(std::max(v, 0.0f), 1.0f);
}

using V3 = std::array<float, 3>;

V3 rrt_odt(V3 v) {
    for (float &c : v) {
        const float a = c * (c + 0.0245786f) - 0.000090537f;
        const float b = c * (0.983729f * c + 0.4329510f) + 0.238081f;
        c = a / b;
    }
    return v;
}

// GLSL mat3(c0, c1, c2) * v: the columns as given.
V3 mul_cols(const float m[9], const V3 &v) {
    return {m[0] * v[0] + m[3] * v[1] + m[6] * v[2], m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
            m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
}

constexpr float kAcesIn[9] = {0.59719f, 0.07600f, 0.02840f, 0.35458f, 0.90834f, 0.13383f, 0.04823f, 0.01566f, 0.83777f};
constexpr float kAcesOut[9] = {1.60475f,  -0.10208f, -0.00327f, -0.53108f, 1.10813f,
                               -0.07276f, -0.07367f, -0.00605f, 1.07602f};

} // namespace

float flicker_value(uint32_t seed, double t, float rate, float amount) {
    if (amount <= 0.0f || rate <= 0.0f)
        return 1.0f;
    const double r = double(rate);
    const float n = 0.5f * value_noise(seed, t * r * 0.25) + 0.3f * value_noise(seed + 1u, t * r * 0.5) +
                    0.2f * value_noise(seed + 2u, t * r);
    return std::max(0.0f, 1.0f + amount * n);
}

std::array<float, 3> ramp_value(const EmissiveRampDesc &ramp, double t) {
    float s = 0.0f;
    if (ramp.duration > 0.0f)
        s = clamp01(float((t - ramp.start) / double(ramp.duration)));
    const float w = std::pow(s, std::max(ramp.curve, 1e-3f));
    const float i = ramp.hot_intensity + (ramp.cool_intensity - ramp.hot_intensity) * w;
    std::array<float, 3> out{};
    for (int k = 0; k < 3; ++k)
        out[k] = (ramp.hot_color[k] + (ramp.cool_color[k] - ramp.hot_color[k]) * w) * i;
    return out;
}

std::array<float, 3> tone_map(ToneMapping op, std::array<float, 3> c, float exposure) {
    for (float &v : c)
        v = std::max(v * exposure, 0.0f);
    switch (op) {
    case ToneMapping::None:
        break;
    case ToneMapping::Neutral: {
        constexpr float start = 0.8f - 0.04f, desaturation = 0.15f;
        const float x = std::min(c[0], std::min(c[1], c[2]));
        const float offset = x < 0.08f ? x - 6.25f * x * x : 0.04f;
        for (float &v : c)
            v -= offset;
        const float peak = std::max(c[0], std::max(c[1], c[2]));
        if (peak >= start) {
            const float d = 1.0f - start;
            const float new_peak = 1.0f - d * d / (peak + d - start);
            for (float &v : c)
                v *= new_peak / peak;
            const float g = 1.0f - 1.0f / (desaturation * (peak - new_peak) + 1.0f);
            for (float &v : c)
                v = v + (new_peak - v) * g;
        }
        break;
    }
    case ToneMapping::Aces: {
        for (float &v : c)
            v /= 0.6f;
        c = mul_cols(kAcesOut, rrt_odt(mul_cols(kAcesIn, c)));
        break;
    }
    }
    for (float &v : c)
        v = clamp01(v);
    return c;
}

std::array<float, 3> inverse_tone_map(ToneMapping op, std::array<float, 3> display, float exposure) {
    for (float &v : display)
        v = clamp01(v);
    exposure = std::max(exposure, 1e-6f);
    if (op == ToneMapping::None) {
        for (float &v : display)
            v /= exposure;
        return display;
    }
    // Channel by channel bisection, a few rounds over the three (Gauss-Seidel): every operator here is
    // non-decreasing in each channel and only weakly coupled across them (the ACES matrices, Neutral's shared
    // offset), and some are flat in places (ACES' toe maps the darkest values to 0), where a Newton step stalls.
    // Where no value reaches `display` (above what the operator can show) it ends at the top of the range.
    std::array<float, 3> x = display;
    for (float &v : x)
        v /= exposure;
    for (int round = 0; round < 6; ++round) {
        for (int k = 0; k < 3; ++k) {
            auto at = [&](float v) {
                auto t = x;
                t[k] = v;
                return tone_map(op, t, exposure)[k];
            };
            float lo = 0.0f, hi = std::max(1e-3f, 2.0f * x[k]);
            while (at(hi) < display[k] && hi < 1e4f)
                hi *= 2.0f;
            for (int it = 0; it < 40; ++it) {
                const float mid = 0.5f * (lo + hi);
                (at(mid) < display[k] ? lo : hi) = mid;
            }
            x[k] = 0.5f * (lo + hi);
        }
    }
    return x;
}

uint16_t float_to_half(float f) {
    const uint32_t x = std::bit_cast<uint32_t>(f);
    const uint32_t sign = (x >> 16) & 0x8000u;
    const int32_t e = int32_t((x >> 23) & 0xff) - 127 + 15;
    uint32_t m = x & 0x7fffffu;
    if (((x >> 23) & 0xff) == 0xff) // inf / nan
        return uint16_t(sign | 0x7c00u | (m ? 0x200u : 0u));
    if (e >= 31)
        return uint16_t(sign | 0x7c00u); // overflow: inf
    if (e <= 0) {
        if (e < -10)
            return uint16_t(sign);
        m |= 0x800000u;
        const uint32_t shift = uint32_t(14 - e);
        uint32_t h = m >> shift;
        if ((m >> (shift - 1)) & 1u) // round half up
            ++h;
        return uint16_t(sign | h);
    }
    uint32_t h = sign | (uint32_t(e) << 10) | (m >> 13);
    if (m & 0x1000u) // round
        ++h;
    return uint16_t(h);
}

float half_to_float(uint16_t h) {
    const uint32_t sign = uint32_t(h & 0x8000u) << 16;
    const uint32_t e = (h >> 10) & 0x1f;
    uint32_t m = h & 0x3ffu;
    if (e == 0) {
        if (m == 0)
            return std::bit_cast<float>(sign);
        // subnormal
        float v = float(m) / 1024.0f * std::ldexp(1.0f, -14);
        return sign ? -v : v;
    }
    if (e == 31)
        return std::bit_cast<float>(sign | 0x7f800000u | (m << 13));
    return std::bit_cast<float>(sign | ((e - 15 + 127) << 23) | (m << 13));
}

std::array<float, 3> env_direction(float u, float v) {
    const float phi = (u - 0.5f) * 2.0f * kPi, theta = v * kPi;
    return {std::sin(theta) * std::sin(phi), std::cos(theta), -std::sin(theta) * std::cos(phi)};
}

std::array<float, 2> env_uv(const std::array<float, 3> &d) {
    const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    const float y = len > 0 ? std::clamp(d[1] / len, -1.0f, 1.0f) : 1.0f;
    return {std::atan2(d[0], -d[2]) / (2.0f * kPi) + 0.5f, std::acos(y) / kPi};
}

EnvImage decode_hdr(const uint8_t *bytes, size_t size) {
    size_t at = 0;
    auto line = [&]() {
        std::string s;
        while (at < size && bytes[at] != '\n')
            s += char(bytes[at++]);
        if (at < size)
            ++at;
        return s;
    };
    const std::string magic = line();
    if (magic.rfind("#?", 0) != 0)
        throw std::runtime_error("not a Radiance .hdr file");
    bool rgbe = false;
    for (;;) {
        if (at >= size)
            throw std::runtime_error(".hdr: no image size");
        const std::string l = line();
        if (l.empty())
            break;
        if (l.find("FORMAT=32-bit_rle_rgbe") != std::string::npos)
            rgbe = true;
        if (l.find("FORMAT=32-bit_rle_xyze") != std::string::npos)
            throw std::runtime_error(".hdr: XYZE is not supported");
    }
    (void) rgbe; // files without a FORMAT line are RGBE too
    const std::string dims = line();
    int w = 0, h = 0;
    char ys = 0, xs = 0, ya = 0, xa = 0;
    if (std::sscanf(dims.c_str(), "%c%c %d %c%c %d", &ys, &ya, &h, &xs, &xa, &w) != 6 || ya != 'Y' || xa != 'X' ||
        w <= 0 || h <= 0 || w > 32768 || h > 32768)
        throw std::runtime_error(".hdr: unsupported image orientation or size '" + dims + "'");
    const bool flip_y = ys == '+', flip_x = xs == '-';
    EnvImage img;
    img.width = uint32_t(w);
    img.height = uint32_t(h);
    img.rgb.assign(size_t(w) * h * 3, 0.0f);
    std::vector<uint8_t> row(size_t(w) * 4);
    for (int y = 0; y < h; ++y) {
        if (at + 4 <= size && w >= 8 && w < 32768 && bytes[at] == 2 && bytes[at + 1] == 2 &&
            ((bytes[at + 2] << 8) | bytes[at + 3]) == w && !(bytes[at + 2] & 0x80)) {
            at += 4;
            for (int c = 0; c < 4; ++c) {
                int x = 0;
                while (x < w) {
                    if (at >= size)
                        throw std::runtime_error(".hdr: truncated");
                    int n = bytes[at++];
                    if (n > 128) {
                        n -= 128;
                        if (at >= size || x + n > w)
                            throw std::runtime_error(".hdr: bad run");
                        const uint8_t v = bytes[at++];
                        for (int k = 0; k < n; ++k)
                            row[size_t(x++) * 4 + c] = v;
                    } else {
                        if (n == 0 || at + n > size || x + n > w)
                            throw std::runtime_error(".hdr: bad run");
                        for (int k = 0; k < n; ++k)
                            row[size_t(x++) * 4 + c] = bytes[at++];
                    }
                }
            }
        } else {
            if (at + size_t(w) * 4 > size)
                throw std::runtime_error(".hdr: truncated");
            std::memcpy(row.data(), bytes + at, size_t(w) * 4);
            at += size_t(w) * 4;
        }
        const int oy = flip_y ? h - 1 - y : y;
        for (int x = 0; x < w; ++x) {
            const uint8_t *p = &row[size_t(x) * 4];
            const int ox = flip_x ? w - 1 - x : x;
            float *o = &img.rgb[(size_t(oy) * w + ox) * 3];
            if (p[3] == 0) {
                o[0] = o[1] = o[2] = 0.0f;
            } else {
                const float f = std::ldexp(1.0f, int(p[3]) - 136);
                o[0] = float(p[0]) * f;
                o[1] = float(p[1]) * f;
                o[2] = float(p[2]) * f;
            }
        }
    }
    return img;
}

EnvImage studio_environment(uint32_t width, uint32_t height) {
    EnvImage img;
    img.width = width;
    img.height = height;
    img.rgb.resize(size_t(width) * height * 3);
    struct Box {
        float az, el, half_az, half_el, intensity;
        std::array<float, 3> tint;
    };
    // Azimuth / elevation in degrees, in the environment frame (+y up, azimuth 0 along -z).
    const Box boxes[] = {
        {35.0f, 32.0f, 22.0f, 11.0f, 9.0f, {1.0f, 0.97f, 0.92f}},  // key
        {-125.0f, 22.0f, 30.0f, 9.0f, 3.5f, {0.92f, 0.96f, 1.0f}}, // fill
        {150.0f, 50.0f, 14.0f, 16.0f, 5.0f, {1.0f, 1.0f, 1.0f}},   // rim
        {0.0f, 85.0f, 180.0f, 6.0f, 2.0f, {1.0f, 1.0f, 1.0f}},     // overhead strip
    };
    for (uint32_t y = 0; y < height; ++y)
        for (uint32_t x = 0; x < width; ++x) {
            const auto d = env_direction((x + 0.5f) / width, (y + 0.5f) / height);
            const float el = std::asin(std::clamp(d[1], -1.0f, 1.0f)) * 180.0f / kPi;
            const float az = std::atan2(d[0], -d[2]) * 180.0f / kPi;
            std::array<float, 3> c{};
            if (d[1] >= 0.0f) {
                const float s = std::sqrt(d[1]);
                const float v = 0.30f + 0.45f * s;
                c = {v * 0.98f, v, v * 1.03f};
            } else {
                const float s = std::min(1.0f, -d[1] * 6.0f);
                const float v = 0.22f - 0.12f * s; // a darker floor, lighter towards the horizon
                c = {v, v * 0.98f, v * 0.95f};
            }
            for (const Box &b : boxes) {
                float daz = std::fabs(az - b.az);
                daz = std::min(daz, 360.0f - daz);
                const float del = std::fabs(el - b.el);
                auto edge = [](float dist, float half) {
                    const float soft = std::max(2.0f, half * 0.25f);
                    return std::clamp((half + soft - dist) / (2.0f * soft), 0.0f, 1.0f);
                };
                const float w = edge(daz, b.half_az) * edge(del, b.half_el);
                for (int k = 0; k < 3; ++k)
                    c[k] += w * b.intensity * b.tint[k];
            }
            std::copy(c.begin(), c.end(), img.rgb.begin() + (size_t(y) * width + x) * 3);
        }
    return img;
}

EnvImage resample(const EnvImage &src, uint32_t width, uint32_t height) {
    if (src.width == 0 || src.height == 0 || src.rgb.size() < size_t(src.width) * src.height * 3)
        throw std::invalid_argument("adacpp.render: empty environment image");
    EnvImage out;
    out.width = width;
    out.height = height;
    out.rgb.assign(size_t(width) * height * 3, 0.0f);
    const double sx = double(src.width) / width, sy = double(src.height) / height;
    for (uint32_t y = 0; y < height; ++y) {
        const uint32_t y0 = uint32_t(std::floor(y * sy)), y1 = std::max(y0 + 1, uint32_t(std::ceil((y + 1) * sy)));
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t x0 = uint32_t(std::floor(x * sx)), x1 = std::max(x0 + 1, uint32_t(std::ceil((x + 1) * sx)));
            double acc[3] = {0, 0, 0};
            uint32_t n = 0;
            for (uint32_t yy = y0; yy < std::min(y1, src.height); ++yy)
                for (uint32_t xx = x0; xx < std::min(x1, src.width); ++xx) {
                    const float *p = &src.rgb[(size_t(yy) * src.width + xx) * 3];
                    for (int k = 0; k < 3; ++k)
                        acc[k] += std::isfinite(p[k]) ? std::max(p[k], 0.0f) : 0.0f;
                    ++n;
                }
            float *o = &out.rgb[(size_t(y) * width + x) * 3];
            for (int k = 0; k < 3; ++k)
                o[k] = n ? float(acc[k] / n) : 0.0f;
        }
    }
    return out;
}

} // namespace fx

} // namespace adacpp::render
