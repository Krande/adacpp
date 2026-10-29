// WGSL for the renderer, embedded so neither host needs a shader toolchain or loose files.
#pragma once

namespace adacpp::render {

// Layout contract with renderer.cpp:
//   Frame     uniform, 112 bytes
//   Instance  storage, 112 bytes each (model, colour, flags, segment base, morph influence, visible base,
//             material: metalness, roughness, specular on/off)
//   segstate  2 x u32 per segment: RGBA8 colour override (alpha 0 = none), flags
//   visible   visible-cluster ids, one region per instance starting at Instance.vis_base
//   vtx       6 x f32 per vertex: position, normal (zero normal = derive a flat one)
//   colors    packed RGBA8 per vertex
//   morph     3 x f32 per vertex
//   prim      triangles: cluster records (4 x u32: vertex offset, triangle offset, triangle count,
//             segment) followed by the cluster vertex lists and the packed local triangles (3 x u8 in
//             one u32, followed by that triangle's index in the original index list: 2 x u32 per
//             triangle); lines/points: the index list followed by the segment of every index entry
//             (at Instance.material.w, bit-cast), so hidden segments drop out without extra draws
inline constexpr const char *kShaderWGSL = R"WGSL(
struct Frame {
    view_proj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    light_dir: vec4<f32>,
    params: vec4<f32>,
};

struct Instance {
    model: mat4x4<f32>,
    color: vec4<f32>,
    flags: u32,
    seg_base: u32,
    morph: f32,
    vis_base: u32,
    material: vec4<f32>, // metalness, roughness, specular (0 = Lambert), lines/points: segment-id offset (u32 bits)
};

const FLAG_VERTEX_COLORS: u32 = 1u;
const FLAG_UNLIT: u32 = 2u;
const FLAG_MORPH_RELATIVE: u32 = 32u;
const FLAG_HAS_MORPH: u32 = 64u;
const CLUSTER_VERTS: u32 = 372u; // 124 triangles x 3

@group(0) @binding(0) var<uniform> frame: Frame;
@group(0) @binding(1) var<storage, read> instances: array<Instance>;
@group(0) @binding(2) var<storage, read> segstate: array<u32>;
@group(0) @binding(3) var<storage, read> visible: array<u32>;

@group(1) @binding(0) var<storage, read> vtx: array<f32>;
@group(1) @binding(1) var<storage, read> colors: array<u32>;
@group(1) @binding(2) var<storage, read> morph: array<f32>;
@group(1) @binding(3) var<storage, read> prim: array<u32>;

struct VOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) @interpolate(flat) flags: u32,
    @location(4) @interpolate(flat) material: vec4<f32>,
    @location(5) @interpolate(flat) pick: vec4<u32>, // instance + 1, primitive, segment, -
};

fn shade_vertex(inst: Instance, vi: u32, seg: i32, ii: u32, primitive: u32) -> VOut {
    var p = vec3<f32>(vtx[vi * 6u], vtx[vi * 6u + 1u], vtx[vi * 6u + 2u]);
    let n = vec3<f32>(vtx[vi * 6u + 3u], vtx[vi * 6u + 4u], vtx[vi * 6u + 5u]);
    if ((inst.flags & FLAG_HAS_MORPH) != 0u) {
        let m = vec3<f32>(morph[vi * 3u], morph[vi * 3u + 1u], morph[vi * 3u + 2u]);
        if ((inst.flags & FLAG_MORPH_RELATIVE) != 0u) {
            p = p + inst.morph * m;
        } else {
            p = mix(p, m, inst.morph);
        }
    }
    let wp = inst.model * vec4<f32>(p, 1.0);
    var c = inst.color;
    if ((inst.flags & FLAG_VERTEX_COLORS) != 0u) {
        c = vec4<f32>(c.rgb * unpack4x8unorm(colors[vi]).rgb, c.a);
    }
    if (seg >= 0) {
        let ov = unpack4x8unorm(segstate[(inst.seg_base + u32(seg)) * 2u]);
        if (ov.a > 0.0) {
            c = vec4<f32>(ov.rgb, c.a);
        }
    }
    var out: VOut;
    out.pos = frame.view_proj * wp;
    out.world = wp.xyz;
    out.normal = (inst.model * vec4<f32>(n, 0.0)).xyz;
    out.color = c;
    out.flags = inst.flags;
    out.material = inst.material;
    out.pick = vec4<u32>(ii + 1u, primitive, u32(max(seg, 0)), 0u);
    return out;
}

@vertex
fn vs_tri(@builtin(vertex_index) vid: u32, @builtin(instance_index) ii: u32) -> VOut {
    let inst = instances[ii];
    let slot = vid / CLUSTER_VERTS;
    let local = vid % CLUSTER_VERTS;
    let tri = local / 3u;
    let corner = local % 3u;
    let cl = visible[inst.vis_base + slot] * 4u;
    let v_off = prim[cl];
    let t_off = prim[cl + 1u];
    let t_cnt = prim[cl + 2u];
    let seg = prim[cl + 3u];
    if (tri >= t_cnt) {
        var dead: VOut;
        dead.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0); // all three corners equal: zero-area, outside the clip volume
        return dead;
    }
    let local_index = (prim[t_off + tri * 2u] >> (corner * 8u)) & 0xffu;
    let vi = prim[v_off + local_index];
    return shade_vertex(inst, vi, i32(seg), ii, prim[t_off + tri * 2u + 1u]);
}

@vertex
fn vs_list(@builtin(vertex_index) vid: u32, @builtin(instance_index) ii: u32) -> VOut {
    let inst = instances[ii];
    let seg = prim[bitcast<u32>(inst.material.w) + vid];
    if ((segstate[(inst.seg_base + seg) * 2u + 1u] & 1u) != 0u) {
        var dead: VOut;
        dead.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0); // hidden segment: both ends collapse outside the clip volume
        return dead;
    }
    return shade_vertex(inst, prim[vid], i32(seg), ii, vid);
}

const PI: f32 = 3.141592653589793;

// three.js MeshStandardMaterial's direct-light BRDF (lights_physical_pars_fragment): GGX distribution,
// height-correlated Smith visibility, Schlick Fresnel with f90 = 1.
fn brdf_ggx(l: vec3<f32>, v: vec3<f32>, n: vec3<f32>, f0: vec3<f32>, roughness: f32) -> vec3<f32> {
    let alpha = roughness * roughness;
    let h = normalize(l + v);
    let ndl = clamp(dot(n, l), 0.0, 1.0);
    let ndv = clamp(dot(n, v), 0.0, 1.0);
    let ndh = clamp(dot(n, h), 0.0, 1.0);
    let vdh = clamp(dot(v, h), 0.0, 1.0);
    let f = f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - vdh, 5.0);
    let a2 = alpha * alpha;
    let gv = ndl * sqrt(a2 + (1.0 - a2) * ndv * ndv);
    let gl = ndv * sqrt(a2 + (1.0 - a2) * ndl * ndl);
    let vis = 0.5 / max(gv + gl, 1e-6);
    let denom = ndh * ndh * (a2 - 1.0) + 1.0;
    let d = a2 / (PI * denom * denom);
    return f * (vis * d);
}

struct PickOut {
    @location(0) id: vec4<u32>,
    @location(1) world: vec4<f32>,
};

@fragment
fn fs_pick(in: VOut) -> PickOut {
    var o: PickOut;
    o.id = in.pick;
    o.world = vec4<f32>(in.world, 1.0);
    return o;
}

fn linear_to_srgb(c: vec3<f32>) -> vec3<f32> {
    let lo = c * 12.92;
    let hi = 1.055 * pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.4)) - 0.055;
    return select(hi, lo, c <= vec3<f32>(0.0031308));
}

@fragment
fn fs_main(in: VOut) -> @location(0) vec4<f32> {
    // Derivatives first, in uniform control flow: the flat normal for geometry without normals.
    let flat_n = cross(dpdx(in.world), dpdy(in.world));
    var rgb = in.color.rgb;
    if ((in.flags & FLAG_UNLIT) == 0u) {
        var n = in.normal;
        if (dot(n, n) < 1e-12) {
            n = flat_n;
        }
        n = normalize(n);
        // The viewer's lights: an ambient light (irradiance params.x) and a directional headlight
        // (intensity params.y) shining along the view direction, i.e. from the camera.
        var v = -frame.light_dir.xyz;
        if (frame.cam_pos.w > 0.5) {
            v = normalize(frame.cam_pos.xyz - in.world);
        }
        if (dot(n, v) < 0.0) {
            n = -n; // double-sided: light the face the camera sees
        }
        let l = -frame.light_dir.xyz;
        let metal = clamp(in.material.x, 0.0, 1.0);
        let rough = clamp(in.material.y, 0.0525, 1.0);
        let diffuse = rgb * (1.0 - metal);
        let irradiance = frame.params.y * clamp(dot(n, l), 0.0, 1.0);
        var out_rgb = diffuse * (frame.params.x + irradiance) / PI;
        if (in.material.z > 0.5) {
            let f0 = mix(vec3<f32>(0.04), rgb, metal);
            out_rgb = out_rgb + irradiance * brdf_ggx(l, v, n, f0, rough);
        }
        rgb = out_rgb;
    }
    return vec4<f32>(linear_to_srgb(rgb), in.color.a);
}
)WGSL";

} // namespace adacpp::render
