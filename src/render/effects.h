// Realistic-visuals parameters for the renderer, as plain data: post-processing (HDR target, tone mapping,
// bloom), image-based lighting, point lights with flicker, emissive ramps and particle emitters. Also the
// math behind them that runs on the CPU (flicker, ramps, tone mapping and its inverse, environment maps),
// kept free of any GPU so it can be tested anywhere. The shaders (shaders.h, effects_shaders.h) evaluate the
// same formulas; keep the two in step.
//
// Time: every effect that changes over time reads the renderer's clock, Renderer::set_time(seconds). It is a
// timeline, not a wall clock: effects are pure functions of it (nothing accumulates frame to frame), so a
// paused, scrubbed or replayed timeline, and a poster rendered at a given time, all show the same thing.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace adacpp::render {

enum class ToneMapping : uint32_t {
    None = 0,    // clamp: the look of the plain (non-HDR) path
    Neutral = 1, // Khronos PBR Neutral: colours kept up to ~0.8, only highlights rolled off
    Aces = 2,    // ACES filmic, as three.js' ACESFilmicToneMapping
};

// Post-processing. `hdr` off (the default) draws as the renderer always has: straight into the target,
// sRGB-encoded by the shaders, nothing changed and nothing extra paid. On, the scene is drawn into a
// half-float (rgba16float) target in linear light, so colours above 1 (emissive, point lights, the sun of an
// environment) survive, and resolved into the target through bloom and tone mapping. The background
// (set_background) is shown as it is either way: it is cleared to what the tone mapping maps onto it.
// Blending of transparent surfaces happens in linear light with `hdr` on (in sRGB-encoded values off).
struct PostSettings {
    bool hdr = false;
    ToneMapping tone_mapping = ToneMapping::Aces;
    float exposure = 1.0f;
    // Bloom (needs `hdr`): what is brighter than `bloom_threshold` (linear, after the exposure; a soft
    // knee `bloom_knee` x threshold wide below it) spreads over a chain of half-resolution levels and is
    // added back with `bloom_strength`. `bloom_radius` 0..1: how much the wider levels weigh against the
    // narrow ones.
    bool bloom = false;
    float bloom_threshold = 1.0f;
    float bloom_knee = 0.5f;
    float bloom_strength = 0.6f;
    float bloom_radius = 0.7f;
};

// Image-based lighting: an environment that every lit surface reflects (prefiltered by roughness, split-sum
// BRDF) and is lit by (its irradiance), on top of the ambient light and headlight of set_lighting(). By
// default the environment is a procedural studio (a soft gradient with a few softboxes); see
// Renderer::set_environment_map() for an equirectangular HDR image.
struct EnvironmentDesc {
    bool enabled = false;
    float intensity = 1.0f;
    // Radians about `up`, counter-clockwise looking down it.
    float rotation = 0.0f;
    // The world's up axis, which the environment's zenith points along (the viewer's world is z-up).
    std::array<float, 3> up{0, 0, 1};
};

// A point light, three.js' PointLight: `intensity` in candela, falling off as 1 / distance^decay and to
// zero at `range` (0 = no cutoff). `flicker` 0..1 modulates the intensity with seeded noise in time
// (flicker_value), `flicker_rate` Hz its fastest variation. On only between `start` and `end` seconds of
// the renderer's clock.
struct PointLightDesc {
    std::array<float, 3> position{0, 0, 0};
    std::array<float, 3> color{1, 1, 1}; // linear RGB
    float intensity = 1.0f;
    float range = 0.0f;
    float decay = 2.0f;
    float flicker = 0.0f;
    float flicker_rate = 8.0f;
    uint32_t seed = 0;
    double start = -1e30;
    double end = 1e30;
    bool enabled = true;
};

// An emissive ramp: a glow that goes from `hot` to `cool` over `duration` seconds from `start` (a part
// cooling down). Linear RGB times intensity, which may well be above 1 (it is light: with PostSettings::hdr it
// is kept, and blooms). The mix weight is ((t - start) / duration)^curve, clamped to [0, 1]: `curve` below 1
// changes fast at first, as a cooling part does. Before `start` it is `hot`; a zero duration is `hot` always.
// Applied to an instance (InstanceDesc::emissive_ramp) or to some segments of one
// (Renderer::set_segment_emissive).
struct EmissiveRampDesc {
    std::array<float, 3> hot_color{1.0f, 0.55f, 0.15f};
    float hot_intensity = 8.0f;
    std::array<float, 3> cool_color{0.6f, 0.04f, 0.0f};
    float cool_intensity = 0.0f;
    double start = 0.0;
    float duration = 1.0f;
    float curve = 1.0f;
};

enum class ParticleBlend : uint32_t {
    Additive = 0, // light: sparks, spatter (colours add up; best with PostSettings::hdr)
    Alpha = 1,    // soft sprites over what is behind: smoke (unsorted, so keep them faint)
};

// A particle emitter: camera-facing billboards drawn instanced on the GPU, depth-tested against the model
// but writing no depth. Particle k is born at start + k / rate (until `end`) at `origin` (moving with the
// emitter's own `velocity` from `start`), within `radius` of it, flying off inside a cone of half-angle
// `spread` around `direction` at `speed` (+- speed_jitter, a share), under `gravity` and a linear `drag`
// (1/s), and lives `lifetime` (shortened by up to lifetime_jitter, a share) seconds. Its size, colour and
// opacity go from the _start to the _end values over its life. `stretch` > 0 draws it as a streak along its
// motion, as long as the distance it covers in `stretch` seconds. Stateless: what is on screen is a function
// of the renderer's time alone (see the note at the top).
struct EmitterDesc {
    std::array<float, 3> origin{0, 0, 0};
    std::array<float, 3> velocity{0, 0, 0};
    std::array<float, 3> direction{0, 0, 1};
    float spread = 0.6f;
    float speed = 2.0f;
    float speed_jitter = 0.5f;
    float rate = 200.0f;
    float lifetime = 0.6f;
    float lifetime_jitter = 0.5f;
    double start = 0.0;
    double end = 1e30;
    std::array<float, 3> gravity{0, 0, -9.81f};
    float drag = 0.0f;
    float radius = 0.0f;
    float size_start = 0.02f;
    float size_end = 0.005f;
    std::array<float, 3> color_start{1.0f, 0.75f, 0.35f}; // linear RGB
    float intensity_start = 6.0f;
    float alpha_start = 1.0f;
    std::array<float, 3> color_end{1.0f, 0.25f, 0.02f};
    float intensity_end = 1.5f;
    float alpha_end = 0.0f;
    float stretch = 0.02f;
    ParticleBlend blend = ParticleBlend::Additive;
    uint32_t seed = 0;
    bool enabled = true;
};

// A smoke preset: slow, growing, fading soft sprites (EmitterDesc with ParticleBlend::Alpha).
EmitterDesc smoke_emitter(const std::array<float, 3> &origin);

namespace fx {

// The flicker multiplier of a light at time `t`: 1 + amount x (smooth noise in [-1, 1], three octaves, the
// fastest at `rate` Hz), never below 0. Deterministic in (seed, t); continuous in t.
float flicker_value(uint32_t seed, double t, float rate, float amount);

// An emissive ramp's colour times intensity at time `t` (see EmissiveRampDesc).
std::array<float, 3> ramp_value(const EmissiveRampDesc &ramp, double t);

// Tone mapping of a linear colour, as the composite shader does it (exposure applied first), in linear
// display values 0..1 (before sRGB encoding).
std::array<float, 3> tone_map(ToneMapping op, std::array<float, 3> rgb, float exposure);
// The linear scene colour tone_map() maps onto `display` (linear, 0..1): what the HDR target is cleared to
// so that the background shows unchanged. Solved iteratively; exact to ~1e-5 where an inverse exists.
std::array<float, 3> inverse_tone_map(ToneMapping op, std::array<float, 3> display, float exposure);

// IEEE half floats, as an rgba16float texture takes them.
uint16_t float_to_half(float f);
float half_to_float(uint16_t h);

// An equirectangular environment image: linear RGB, row 0 at the zenith; u = 0.5 looks along the
// environment's -z (see env_direction).
struct EnvImage {
    uint32_t width = 0, height = 0;
    std::vector<float> rgb; // 3 per texel
};
// The direction (environment frame: +y up) texel centre (u, v) in [0, 1]^2 looks along, and back.
std::array<float, 3> env_direction(float u, float v);
std::array<float, 2> env_uv(const std::array<float, 3> &dir);
// Radiance .hdr (RGBE, uncompressed or RLE) bytes. Throws std::runtime_error on anything it cannot read.
EnvImage decode_hdr(const uint8_t *bytes, size_t size);
// The procedural studio: a soft overhead gradient, a darker floor and three softboxes.
EnvImage studio_environment(uint32_t width = 512, uint32_t height = 256);
// Box-filtered to width x height (any size in, power-of-two sizes out are what the renderer uses).
EnvImage resample(const EnvImage &src, uint32_t width, uint32_t height);

} // namespace fx

} // namespace adacpp::render
