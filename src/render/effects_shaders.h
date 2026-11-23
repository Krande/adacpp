// WGSL for the realistic-visuals passes (effects.h): the environment's prefiltering, bloom and the HDR
// composite, and particles. The lighting itself (point lights, image-based lighting, emissive) is in the
// main shader, shaders.h.
#pragma once

namespace adacpp::render {

// The environment, once per environment (effects_gpu.cpp Environment): compute only.
//   cs_prefilter   one mip of the radiance prefiltered for roughness ep.x (GGX importance sampling, sampling
//                  the source's mips by each sample's solid angle: filtered importance sampling)
//   cs_irradiance  the irradiance (cosine-weighted integral) of every direction, from a small source mip
//   cs_brdf        the split-sum BRDF table: x = NdotV, y = roughness -> (F0 scale, F0 bias)
// Directions are in the environment frame (+y up), equirectangular as fx::env_direction / env_uv.
inline constexpr const char *kEnvWGSL = R"WGSL(
const PI: f32 = 3.141592653589793;

struct EnvParams {
    p: vec4<f32>,  // x: roughness, y: source mip for cs_irradiance, z: samples
};

@group(0) @binding(0) var<uniform> ep: EnvParams;
@group(0) @binding(1) var src: texture_2d<f32>;
@group(0) @binding(2) var samp: sampler;
@group(0) @binding(3) var dst: texture_storage_2d<rgba16float, write>;

fn dir_of(uv: vec2<f32>) -> vec3<f32> {
    let phi = (uv.x - 0.5) * 2.0 * PI;
    let theta = uv.y * PI;
    return vec3<f32>(sin(theta) * sin(phi), cos(theta), -sin(theta) * cos(phi));
}

fn uv_of(d: vec3<f32>) -> vec2<f32> {
    return vec2<f32>(atan2(d.x, -d.z) / (2.0 * PI) + 0.5, acos(clamp(d.y, -1.0, 1.0)) / PI);
}

fn hammersley(i: u32, n: u32) -> vec2<f32> {
    return vec2<f32>(f32(i) / f32(n), f32(reverseBits(i)) * 2.3283064365386963e-10);
}

// A GGX-distributed half vector around `n` (alpha = roughness^2).
fn sample_ggx(xi: vec2<f32>, n: vec3<f32>, a: f32) -> vec3<f32> {
    let phi = 2.0 * PI * xi.x;
    let ct = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
    let st = sqrt(max(1.0 - ct * ct, 0.0));
    let up = select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(1.0, 0.0, 0.0), abs(n.z) > 0.999);
    let tx = normalize(cross(up, n));
    let ty = cross(n, tx);
    return normalize(tx * (cos(phi) * st) + ty * (sin(phi) * st) + n * ct);
}

@compute @workgroup_size(8, 8)
fn cs_prefilter(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(dst);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let uv = (vec2<f32>(gid.xy) + 0.5) / vec2<f32>(size);
    let rough = ep.p.x;
    if (rough <= 0.0) {
        textureStore(dst, gid.xy, vec4<f32>(textureSampleLevel(src, samp, uv, 0.0).rgb, 1.0));
        return;
    }
    let n = dir_of(uv);
    let a = rough * rough;
    let src_size = vec2<f32>(textureDimensions(src, 0));
    let texel_sa = 4.0 * PI / (src_size.x * src_size.y);
    let count = u32(ep.p.z);
    var sum = vec3<f32>(0.0);
    var weight = 0.0;
    for (var i = 0u; i < count; i = i + 1u) {
        let h = sample_ggx(hammersley(i, count), n, a);
        let l = 2.0 * dot(n, h) * h - n;
        let ndl = dot(n, l);
        if (ndl > 0.0) {
            let ndh = max(dot(n, h), 0.0);
            let d = a * a / (PI * pow(ndh * ndh * (a * a - 1.0) + 1.0, 2.0));
            let pdf = d / 4.0;
            let sample_sa = 1.0 / (f32(count) * pdf + 1e-6);
            let lod = max(0.5 * log2(sample_sa / texel_sa) + 1.0, 0.0);
            sum = sum + textureSampleLevel(src, samp, uv_of(l), lod).rgb * ndl;
            weight = weight + ndl;
        }
    }
    textureStore(dst, gid.xy, vec4<f32>(sum / max(weight, 1e-6), 1.0));
}

@compute @workgroup_size(8, 8)
fn cs_irradiance(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(dst);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let n = dir_of((vec2<f32>(gid.xy) + 0.5) / vec2<f32>(size));
    let level = u32(ep.p.y);
    let s = textureDimensions(src, level);
    var sum = vec3<f32>(0.0);
    for (var y = 0u; y < s.y; y = y + 1u) {
        let theta = (f32(y) + 0.5) / f32(s.y) * PI;
        let sa = (2.0 * PI / f32(s.x)) * (PI / f32(s.y)) * sin(theta);
        for (var x = 0u; x < s.x; x = x + 1u) {
            let d = dir_of((vec2<f32>(f32(x), f32(y)) + 0.5) / vec2<f32>(s));
            let c = max(dot(n, d), 0.0);
            if (c > 0.0) {
                sum = sum + textureLoad(src, vec2<u32>(x, y), level).rgb * (c * sa);
            }
        }
    }
    textureStore(dst, gid.xy, vec4<f32>(sum, 1.0));
}

@compute @workgroup_size(8, 8)
fn cs_brdf(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(dst);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let ndv = (f32(gid.x) + 0.5) / f32(size.x);
    let rough = (f32(gid.y) + 0.5) / f32(size.y);
    let a = rough * rough;
    let v = vec3<f32>(sqrt(1.0 - ndv * ndv), 0.0, ndv);
    let n = vec3<f32>(0.0, 0.0, 1.0);
    let count = 512u;
    var acc = vec2<f32>(0.0);
    for (var i = 0u; i < count; i = i + 1u) {
        let h = sample_ggx(hammersley(i, count), n, a);
        let l = 2.0 * dot(v, h) * h - v;
        let ndl = clamp(l.z, 0.0, 1.0);
        let ndh = clamp(h.z, 0.0, 1.0);
        let vdh = clamp(dot(v, h), 0.0, 1.0);
        if (ndl > 0.0) {
            let a2 = a * a;
            let vis = 0.5 / (ndl * sqrt(ndv * ndv * (1.0 - a2) + a2) + ndv * sqrt(ndl * ndl * (1.0 - a2) + a2));
            let g = vis * 4.0 * ndl * vdh / max(ndh, 1e-6);
            let fc = pow(1.0 - vdh, 5.0);
            acc = acc + vec2<f32>((1.0 - fc) * g, fc * g);
        }
    }
    textureStore(dst, gid.xy, vec4<f32>(acc / f32(count), 0.0, 1.0));
}
)WGSL";

// Bloom and the HDR composite (effects_gpu.cpp PostChain): full-screen triangles.
//   fs_prefilter  the HDR image down to half size (13 taps), kept where brighter than the threshold
//   fs_down       one bloom level to the next, half its size (13 taps)
//   fs_up         a level up into the one above it, a 3 x 3 tent, added (blend one + one) x radius
//   fs_composite  HDR + bloom, exposed, tone mapped and sRGB-encoded into the host's target
inline constexpr const char *kPostWGSL = R"WGSL(
struct Post {
    a: vec4<f32>, // exposure, bloom threshold, knee (x threshold), bloom strength
    b: vec4<f32>, // bloom radius, tone mapping (ToneMapping), bloom on (1/0), 0
};

@group(0) @binding(0) var<uniform> post: Post;
@group(0) @binding(1) var src: texture_2d<f32>;
@group(0) @binding(2) var samp: sampler;
@group(0) @binding(3) var bloom: texture_2d<f32>;

struct FOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_full(@builtin(vertex_index) vid: u32) -> FOut {
    let p = vec2<f32>(f32(vid & 1u) * 4.0 - 1.0, f32(vid >> 1u) * 4.0 - 1.0);
    var o: FOut;
    o.pos = vec4<f32>(p, 0.0, 1.0);
    o.uv = vec2<f32>(p.x * 0.5 + 0.5, 0.5 - p.y * 0.5);
    return o;
}

// The 13-tap downsample of Jimenez, "Next generation post processing in Call of Duty: Advanced Warfare".
fn down13(uv: vec2<f32>) -> vec3<f32> {
    let t = 1.0 / vec2<f32>(textureDimensions(src, 0));
    let a = textureSampleLevel(src, samp, uv + t * vec2<f32>(-2.0, -2.0), 0.0).rgb;
    let b = textureSampleLevel(src, samp, uv + t * vec2<f32>(0.0, -2.0), 0.0).rgb;
    let c = textureSampleLevel(src, samp, uv + t * vec2<f32>(2.0, -2.0), 0.0).rgb;
    let d = textureSampleLevel(src, samp, uv + t * vec2<f32>(-2.0, 0.0), 0.0).rgb;
    let e = textureSampleLevel(src, samp, uv, 0.0).rgb;
    let f = textureSampleLevel(src, samp, uv + t * vec2<f32>(2.0, 0.0), 0.0).rgb;
    let g = textureSampleLevel(src, samp, uv + t * vec2<f32>(-2.0, 2.0), 0.0).rgb;
    let h = textureSampleLevel(src, samp, uv + t * vec2<f32>(0.0, 2.0), 0.0).rgb;
    let i = textureSampleLevel(src, samp, uv + t * vec2<f32>(2.0, 2.0), 0.0).rgb;
    let j = textureSampleLevel(src, samp, uv + t * vec2<f32>(-1.0, -1.0), 0.0).rgb;
    let k = textureSampleLevel(src, samp, uv + t * vec2<f32>(1.0, -1.0), 0.0).rgb;
    let l = textureSampleLevel(src, samp, uv + t * vec2<f32>(-1.0, 1.0), 0.0).rgb;
    let m = textureSampleLevel(src, samp, uv + t * vec2<f32>(1.0, 1.0), 0.0).rgb;
    return e * 0.125 + (a + c + g + i) * 0.03125 + (b + d + f + h) * 0.0625 + (j + k + l + m) * 0.125;
}

@fragment
fn fs_prefilter(in: FOut) -> @location(0) vec4<f32> {
    let c = min(down13(in.uv), vec3<f32>(60000.0));
    // Soft threshold on the exposed brightness (the knee keeps the cut from showing as an edge).
    let br = max(c.r, max(c.g, c.b)) * post.a.x;
    let thr = post.a.y;
    let knee = max(thr * post.a.z, 1e-5);
    var soft = clamp(br - thr + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    let w = max(soft, br - thr) / max(br, 1e-5);
    return vec4<f32>(c * w, 1.0);
}

@fragment
fn fs_down(in: FOut) -> @location(0) vec4<f32> {
    return vec4<f32>(down13(in.uv), 1.0);
}

@fragment
fn fs_up(in: FOut) -> @location(0) vec4<f32> {
    let t = 1.0 / vec2<f32>(textureDimensions(src, 0));
    var c = textureSampleLevel(src, samp, in.uv, 0.0).rgb * 4.0;
    c = c + (textureSampleLevel(src, samp, in.uv + t * vec2<f32>(-1.0, 0.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(1.0, 0.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(0.0, -1.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(0.0, 1.0), 0.0).rgb) * 2.0;
    c = c + (textureSampleLevel(src, samp, in.uv + t * vec2<f32>(-1.0, -1.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(1.0, -1.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(-1.0, 1.0), 0.0).rgb +
             textureSampleLevel(src, samp, in.uv + t * vec2<f32>(1.0, 1.0), 0.0).rgb);
    return vec4<f32>(c / 16.0 * post.b.x, 1.0);
}

fn rrt_odt(v: vec3<f32>) -> vec3<f32> {
    let a = v * (v + 0.0245786) - 0.000090537;
    let b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

// fx::tone_map in effects.cpp.
fn tone_map(c_in: vec3<f32>, op: u32) -> vec3<f32> {
    var c = max(c_in, vec3<f32>(0.0));
    if (op == 1u) {
        let start = 0.8 - 0.04;
        let x = min(c.r, min(c.g, c.b));
        let offset = select(0.04, x - 6.25 * x * x, x < 0.08);
        c = c - offset;
        let peak = max(c.r, max(c.g, c.b));
        if (peak >= start) {
            let d = 1.0 - start;
            let new_peak = 1.0 - d * d / (peak + d - start);
            c = c * (new_peak / peak);
            let g = 1.0 - 1.0 / (0.15 * (peak - new_peak) + 1.0);
            c = mix(c, vec3<f32>(new_peak), g);
        }
    } else if (op == 2u) {
        let aces_in = mat3x3<f32>(vec3<f32>(0.59719, 0.07600, 0.02840), vec3<f32>(0.35458, 0.90834, 0.13383),
                                  vec3<f32>(0.04823, 0.01566, 0.83777));
        let aces_out = mat3x3<f32>(vec3<f32>(1.60475, -0.10208, -0.00327), vec3<f32>(-0.53108, 1.10813, -0.07276),
                                   vec3<f32>(-0.07367, -0.00605, 1.07602));
        c = aces_out * rrt_odt(aces_in * (c / 0.6));
    }
    return clamp(c, vec3<f32>(0.0), vec3<f32>(1.0));
}

fn linear_to_srgb(c: vec3<f32>) -> vec3<f32> {
    let lo = c * 12.92;
    let hi = 1.055 * pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.4)) - 0.055;
    return select(hi, lo, c <= vec3<f32>(0.0031308));
}

@fragment
fn fs_composite(in: FOut) -> @location(0) vec4<f32> {
    let hdr = textureLoad(src, vec2<i32>(in.pos.xy), 0);
    var c = hdr.rgb;
    if (post.b.z > 0.5) {
        c = c + textureSampleLevel(bloom, samp, in.uv, 0.0).rgb * post.a.w;
    }
    return vec4<f32>(linear_to_srgb(tone_map(c * post.a.x, u32(post.b.y))), hdr.a);
}
)WGSL";

// Particles (effects.h EmitterDesc), stateless: particle k of an emitter is born at start + k / rate, so slot
// j of its kMaxParticlesPerEmitter draws the latest particle k = j (mod capacity) born by now, if it still
// lives. One instanced draw of 6 vertices per emitter, the emitter in instance_index / MAX_PER_EMITTER (the
// draw's first instance), the slot in its remainder.
//
// Layout contract with effects_gpu.cpp:
//   PFrame   uniform, 160 bytes
//   Emitter  storage, 144 bytes each
inline constexpr const char *kParticleWGSL = R"WGSL(
const PI: f32 = 3.141592653589793;
const MAX_PER_EMITTER: u32 = {{MAX_PER_EMITTER}}u;

struct PFrame {
    view: mat4x4<f32>,
    proj: mat4x4<f32>, // WebGPU clip space, depth reversed
    time: vec4<f32>,   // x: the renderer's time
    info: vec4<u32>,   // x: 1 = write linear light (the HDR target)
};

struct Emitter {
    origin: vec4<f32>,  // xyz, start
    motion: vec4<f32>,  // the emitter's velocity, end
    dir: vec4<f32>,     // axis, cos(spread)
    gravity: vec4<f32>, // xyz, drag
    speed: vec4<f32>,   // speed, speed jitter, lifetime, lifetime jitter
    c0: vec4<f32>,      // colour x intensity, alpha at birth
    c1: vec4<f32>,      // at death
    shape: vec4<f32>,   // size at birth, at death, stretch, spawn radius
    misc: vec4<u32>,    // rate (f32 bits), capacity, seed, blend
};

@group(0) @binding(0) var<uniform> pf: PFrame;
@group(0) @binding(1) var<storage, read> emitters: array<Emitter>;

struct POut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) @interpolate(flat) color: vec4<f32>,
};

fn pcg(v: u32) -> u32 {
    let s = v * 747796405u + 2891336453u;
    let w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}

fn rnd(h: ptr<function, u32>) -> f32 {
    *h = pcg(*h);
    return f32(*h) * 2.3283064365386963e-10;
}

@vertex
fn vs_particle(@builtin(vertex_index) vid: u32, @builtin(instance_index) iid: u32) -> POut {
    var out: POut;
    out.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0); // dead: outside the clip volume
    let e = emitters[iid / MAX_PER_EMITTER];
    let j = iid % MAX_PER_EMITTER;
    let rate = bitcast<f32>(e.misc.x);
    let cap = e.misc.y;
    if (j >= cap || rate <= 0.0) {
        return out;
    }
    let t = pf.time.x - e.origin.w; // since the emitter started
    if (t < 0.0) {
        return out;
    }
    let born = min(floor(t * rate), min(floor((e.motion.w - e.origin.w) * rate), 2.0e9));
    let kn = i32(born);
    if (kn < i32(j)) {
        return out;
    }
    let k = kn - (kn - i32(j)) % i32(cap);
    let birth = f32(k) / rate;
    let age = t - birth;
    var h = pcg(e.misc.z ^ pcg(u32(k)));
    let life = e.speed.z * (1.0 - e.speed.w * rnd(&h));
    if (age < 0.0 || age > life || life <= 0.0) {
        return out;
    }
    let f = age / life;

    // Direction in the cone, speed, where in the spawn sphere.
    let ct = mix(1.0, e.dir.w, rnd(&h));
    let st = sqrt(max(1.0 - ct * ct, 0.0));
    let phi = 2.0 * PI * rnd(&h);
    let axis = normalize(e.dir.xyz);
    let up = select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(1.0, 0.0, 0.0), abs(axis.z) > 0.999);
    let tx = normalize(cross(up, axis));
    let ty = cross(axis, tx);
    let dir = tx * (cos(phi) * st) + ty * (sin(phi) * st) + axis * ct;
    let speed = e.speed.x * (1.0 + e.speed.y * (2.0 * rnd(&h) - 1.0));
    let v0 = dir * speed;
    let sz = 2.0 * rnd(&h) - 1.0;
    let sphi = 2.0 * PI * rnd(&h);
    let sr = e.shape.w * pow(rnd(&h), 1.0 / 3.0);
    let sp = vec3<f32>(sqrt(max(1.0 - sz * sz, 0.0)) * cos(sphi), sqrt(max(1.0 - sz * sz, 0.0)) * sin(sphi), sz);
    let p0 = e.origin.xyz + e.motion.xyz * birth + sp * sr;

    // Ballistic with linear drag, in closed form.
    let g = e.gravity.xyz;
    let kd = e.gravity.w;
    var p: vec3<f32>;
    var vel: vec3<f32>;
    if (kd > 1e-4) {
        let ek = exp(-kd * age);
        p = p0 + (v0 - g / kd) * ((1.0 - ek) / kd) + g * (age / kd);
        vel = (v0 - g / kd) * ek + g / kd;
    } else {
        p = p0 + v0 * age + 0.5 * g * age * age;
        vel = v0 + g * age;
    }

    let size = mix(e.shape.x, e.shape.y, f);
    let corner = vid % 6u;
    let q = vec2<f32>(select(-1.0, 1.0, corner == 1u || corner == 2u || corner == 4u),
                      select(-1.0, 1.0, corner == 2u || corner == 4u || corner == 5u));
    let mv = pf.view * vec4<f32>(p, 1.0);
    let vv = (pf.view * vec4<f32>(vel, 0.0)).xy;
    var offset = q * (0.5 * size);
    let streak = length(vv) * e.shape.z;
    if (e.shape.z > 0.0 && streak > 1e-6) {
        // A streak trailing behind the particle: from where it was `stretch` seconds ago to where it is.
        let along = vv / length(vv);
        let side = vec2<f32>(-along.y, along.x);
        offset = along * (q.x * 0.5 * (size + streak) - 0.5 * streak) + side * (q.y * 0.5 * size);
    }
    out.pos = pf.proj * vec4<f32>(mv.xy + offset, mv.z, 1.0);
    out.uv = q;
    out.color = vec4<f32>(mix(e.c0.rgb, e.c1.rgb, f), mix(e.c0.a, e.c1.a, f));
    return out;
}

fn linear_to_srgb(c: vec3<f32>) -> vec3<f32> {
    let lo = c * 12.92;
    let hi = 1.055 * pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.4)) - 0.055;
    return select(hi, lo, c <= vec3<f32>(0.0031308));
}

// Additive: light, blended one + one (alpha untouched).
@fragment
fn fs_additive(in: POut) -> @location(0) vec4<f32> {
    let r2 = dot(in.uv, in.uv);
    if (r2 >= 1.0) {
        discard;
    }
    let fall = (1.0 - r2) * (1.0 - r2);
    let c = in.color.rgb * (in.color.a * fall);
    if (pf.info.x != 0u) {
        return vec4<f32>(c, 0.0);
    }
    return vec4<f32>(linear_to_srgb(c), 0.0);
}

// Alpha: a soft sprite over what is behind (smoke).
@fragment
fn fs_alpha(in: POut) -> @location(0) vec4<f32> {
    let r2 = dot(in.uv, in.uv);
    if (r2 >= 1.0) {
        discard;
    }
    let a = in.color.a * (1.0 - r2) * (1.0 - r2);
    if (pf.info.x != 0u) {
        return vec4<f32>(in.color.rgb, a);
    }
    return vec4<f32>(linear_to_srgb(in.color.rgb), a);
}
)WGSL";

} // namespace adacpp::render
