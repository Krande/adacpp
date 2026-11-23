// The CPU half of the renderer's realistic visuals (src/render/effects.h): flicker noise, emissive ramps, tone
// mapping and its inverse, half floats, the Radiance .hdr reader and the procedural environment. No GPU: runs
// anywhere (pixi run -e render test-render). What the shaders do with the same formulas is checked in
// test_render_scene.cpp.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "effects.h"

using namespace adacpp::render;

namespace {

int failures = 0, checks = 0;

void check(bool ok, const char *what, int line) {
    ++checks;
    if (!ok) {
        ++failures;
        std::fprintf(stderr, "FAILED line %d: %s\n", line, what);
    }
}
#define CHECK(cond) check((cond), #cond, __LINE__)

bool close(float a, float b, float tol) {
    return std::fabs(a - b) <= tol;
}

void test_flicker() {
    CHECK(fx::flicker_value(1, 3.0, 10.0f, 0.0f) == 1.0f); // no flicker
    float lo = 10, hi = -10, sum = 0, last = fx::flicker_value(5, 0.0, 12.0f, 0.5f), max_step = 0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const double t = i * 0.001;
        const float v = fx::flicker_value(5, t, 12.0f, 0.5f);
        CHECK(v == fx::flicker_value(5, t, 12.0f, 0.5f)); // deterministic
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        sum += v;
        max_step = std::max(max_step, std::fabs(v - last));
        last = v;
    }
    CHECK(lo >= 0.5f - 1e-5f && hi <= 1.5f + 1e-5f); // 1 +- amount
    CHECK(hi - lo > 0.3f);                           // it does move
    CHECK(close(sum / n, 1.0f, 0.1f));               // around the light's intensity
    CHECK(max_step < 0.05f);                         // continuous: 1 ms steps change it a little
    // Another seed, another flicker; never below zero, however strong.
    int same = 0;
    for (int i = 0; i < 100; ++i)
        same += fx::flicker_value(5, i * 0.1, 12.0f, 0.5f) == fx::flicker_value(6, i * 0.1, 12.0f, 0.5f);
    CHECK(same < 5);
    for (int i = 0; i < 1000; ++i)
        CHECK(fx::flicker_value(9, i * 0.013, 30.0f, 3.0f) >= 0.0f);
}

void test_ramp() {
    EmissiveRampDesc r;
    r.hot_color = {1, 0.5f, 0};
    r.hot_intensity = 10;
    r.cool_color = {0.5f, 0, 0};
    r.cool_intensity = 0;
    r.start = 2.0;
    r.duration = 4.0f;
    r.curve = 1.0f;
    auto v = fx::ramp_value(r, 0.0); // before the start: hot
    CHECK(close(v[0], 10, 1e-5f) && close(v[1], 5, 1e-5f) && close(v[2], 0, 1e-6f));
    v = fx::ramp_value(r, 4.0); // half way: colour and intensity half way
    CHECK(close(v[0], 0.75f * 5, 1e-4f) && close(v[1], 0.25f * 5, 1e-4f));
    v = fx::ramp_value(r, 100.0); // after: cool
    CHECK(close(v[0], 0, 1e-6f));
    // Monotonic for a ramp to black, and a fast early drop with curve < 1.
    float prev = 1e9f;
    for (int i = 0; i <= 40; ++i) {
        const float x = fx::ramp_value(r, 2.0 + i * 0.1)[0];
        CHECK(x <= prev + 1e-6f);
        prev = x;
    }
    EmissiveRampDesc fast = r;
    fast.curve = 0.3f;
    CHECK(fx::ramp_value(fast, 3.0)[0] < fx::ramp_value(r, 3.0)[0]);
    r.duration = 0.0f; // constant
    CHECK(close(fx::ramp_value(r, 50.0)[0], 10, 1e-5f));
}

void test_tone_mapping() {
    // None: clamp.
    auto c = fx::tone_map(ToneMapping::None, {0.25f, 1.5f, -1.0f}, 2.0f);
    CHECK(close(c[0], 0.5f, 1e-6f) && c[1] == 1.0f && c[2] == 0.0f);
    // Neutral keeps mid tones (but for its small offset) and rolls off highlights; ACES compresses everything.
    c = fx::tone_map(ToneMapping::Neutral, {0.5f, 0.5f, 0.5f}, 1.0f);
    CHECK(close(c[0], 0.46f, 1e-4f));
    for (ToneMapping op : {ToneMapping::Neutral, ToneMapping::Aces}) {
        float prev = -1;
        for (float x = 0; x < 50; x *= 1.2f, x += 0.001f) {
            const float y = fx::tone_map(op, {x, x, x}, 1.0f)[1];
            CHECK(y >= prev - 1e-6f && y <= 1.0f);
            prev = y;
        }
        CHECK(fx::tone_map(op, {100, 100, 100}, 1.0f)[0] > 0.95f);
    }
    // three.js' ACESFilmicToneMapping of mid grey at exposure 1.
    c = fx::tone_map(ToneMapping::Aces, {0.18f, 0.18f, 0.18f}, 1.0f);
    CHECK(c[0] > 0.2f && c[0] < 0.35f);
    // The inverse: what the HDR target is cleared to so a background shows unchanged.
    const std::vector<std::array<float, 3>> backgrounds = {{0.0f, 0.0f, 0.0f},
                                                           {0.0423f, 0.0423f, 0.0423f},
                                                           {0.003f, 0.007f, 0.013f},
                                                           {0.8f, 0.85f, 0.9f},
                                                           {0.9f, 0.2f, 0.05f}};
    for (ToneMapping op : {ToneMapping::None, ToneMapping::Neutral, ToneMapping::Aces})
        for (float exposure : {0.5f, 1.0f, 2.0f})
            for (const auto &bg : backgrounds) {
                const auto x = fx::inverse_tone_map(op, bg, exposure);
                const auto y = fx::tone_map(op, x, exposure);
                for (int k = 0; k < 3; ++k)
                    CHECK(close(y[k], bg[k], 2e-4f));
            }
}

void test_half() {
    for (float f : {0.0f, 1.0f, -2.5f, 0.333f, 1000.0f, 6.0e-5f, 65504.0f}) {
        const float back = fx::half_to_float(fx::float_to_half(f));
        CHECK(close(back, f, std::fabs(f) * 1e-3f + 1e-7f));
    }
    CHECK(fx::float_to_half(1.0f) == 0x3c00);
    CHECK(std::isinf(fx::half_to_float(fx::float_to_half(1e6f))));
}

void test_hdr_file() {
    // A 2 x 1 image, flat RGBE: (1, 0.5, 0.25) and (4, 0, 0).
    std::string text = "#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 1 +X 2\n";
    std::vector<uint8_t> bytes(text.begin(), text.end());
    const uint8_t px[8] = {128, 64, 32, 129, 128, 0, 0, 131};
    bytes.insert(bytes.end(), px, px + 8);
    const auto img = fx::decode_hdr(bytes.data(), bytes.size());
    CHECK(img.width == 2 && img.height == 1);
    CHECK(close(img.rgb[0], 1.0f, 1e-6f) && close(img.rgb[1], 0.5f, 1e-6f) && close(img.rgb[2], 0.25f, 1e-6f));
    CHECK(close(img.rgb[3], 4.0f, 1e-6f) && img.rgb[4] == 0.0f);
    // Run-length encoded scanlines: 8 wide, one run of 8 per channel.
    text = "#?RGBE\n\n-Y 1 +X 8\n";
    bytes.assign(text.begin(), text.end());
    const uint8_t rle[] = {2, 2, 0, 8, 128 + 8, 128, 128 + 8, 128, 128 + 8, 0, 128 + 8, 130};
    bytes.insert(bytes.end(), rle, rle + sizeof(rle));
    const auto rimg = fx::decode_hdr(bytes.data(), bytes.size());
    CHECK(rimg.width == 8 && close(rimg.rgb[7 * 3], 2.0f, 1e-6f) && close(rimg.rgb[7 * 3 + 1], 2.0f, 1e-6f));
    bool threw = false;
    try {
        const uint8_t junk[4] = {1, 2, 3, 4};
        fx::decode_hdr(junk, 4);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    CHECK(threw);
}

void test_environment() {
    // Directions and coordinates are inverses; the zenith is row 0.
    for (float u : {0.1f, 0.5f, 0.77f})
        for (float v : {0.2f, 0.5f, 0.9f}) {
            const auto uv = fx::env_uv(fx::env_direction(u, v));
            CHECK(close(uv[0], u, 1e-4f) && close(uv[1], v, 1e-4f));
        }
    CHECK(close(fx::env_direction(0.5f, 0.0f)[1], 1.0f, 1e-6f));
    const auto studio = fx::studio_environment(64, 32);
    CHECK(studio.rgb.size() == 64 * 32 * 3);
    float top = 0, bottom = 0, peak = 0;
    for (uint32_t x = 0; x < 64; ++x) {
        top += studio.rgb[(5 * 64 + x) * 3 + 1];
        bottom += studio.rgb[(28 * 64 + x) * 3 + 1];
    }
    for (float v : studio.rgb) {
        CHECK(v >= 0.0f && std::isfinite(v));
        peak = std::max(peak, v);
    }
    CHECK(top > bottom); // a lit ceiling over a darker floor
    CHECK(peak > 3.0f);  // and softboxes much brighter than both: what metal reflects
    const auto half = fx::resample(studio, 32, 16);
    double a = 0, b = 0;
    for (float v : studio.rgb)
        a += v;
    for (float v : half.rgb)
        b += v;
    CHECK(close(float(a / studio.rgb.size()), float(b / half.rgb.size()), 1e-3f)); // a box filter keeps the mean
}

void test_presets() {
    const EmitterDesc smoke = smoke_emitter({1, 2, 3});
    CHECK(smoke.blend == ParticleBlend::Alpha && smoke.origin[2] == 3.0f && smoke.size_end > smoke.size_start);
}

} // namespace

int main() {
    test_flicker();
    test_ramp();
    test_tone_mapping();
    test_half();
    test_hdr_file();
    test_environment();
    test_presets();
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
