// WGSL for the renderer, embedded so neither host needs a shader toolchain or loose files.
#pragma once

namespace adacpp::render {

// Layout contract with renderer.cpp:
//   Frame     uniform, 112 bytes
//   Instance  storage, 112 bytes each (model, colour, flags, segment base, morph influence, visible base,
//             material: metalness, roughness, specular on/off)
//   segstate  2 x u32 per segment: RGBA8 colour override (alpha 0 = none), flags
//   visible   visible clusters as (instance, cluster) pairs, one region per instance starting at
//             Instance.vis_base; a draw starts at vertex vis_base x CLUSTER_VERTS, so vs_tri finds its
//             slot from the vertex index alone (indirect draws cannot rely on firstInstance)
//   vtx       6 x f32 per vertex: position, normal (zero normal = derive a flat one)
//   colors    packed RGBA8 per vertex
//   morph     3 x f32 per vertex
//   prim      triangles: cluster records (4 x u32: vertex offset, triangle offset, triangle count,
//             segment) followed by the cluster vertex lists and the packed local triangles (3 x u8 in
//             one u32, followed by that triangle's index in the original index list and its segment:
//             3 x u32 per triangle); lines/points: the index list followed by the segment of every index entry
//             (at Instance.material.w, bit-cast), so hidden segments drop out without extra draws
inline constexpr const char *kShaderWGSL = R"WGSL(
struct Frame {
    view_proj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    light_dir: vec4<f32>,
    params: vec4<f32>, // ambient, headlight, target width, target height (pixels)
};

struct Instance {
    model: mat4x4<f32>,
    color: vec4<f32>,
    flags: u32,
    seg_base: u32,
    morph: f32,
    vis_base: u32,
    material: vec4<f32>, // metalness, roughness, specular (0 = Lambert), lines/points: segment-id offset (u32 bits)
                         // lines/points: x = width / diameter in pixels
};

const FLAG_VERTEX_COLORS: u32 = 1u;
const FLAG_UNLIT: u32 = 2u;
const FLAG_MORPH_RELATIVE: u32 = 32u;
const FLAG_HAS_MORPH: u32 = 64u;
const FLAG_POINTS: u32 = 1024u; // internal: a points instance (vs_wide draws squares, not segments)
const CLUSTER_VERTS: u32 = {{CLUSTER_VERTS}}u; // Renderer::kClusterMaxTriangles x 3, filled in at load

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
fn vs_tri(@builtin(vertex_index) vid: u32) -> VOut {
    let slot = vid / CLUSTER_VERTS;
    let ii = visible[slot * 2u];
    let inst = instances[ii];
    let local = vid % CLUSTER_VERTS;
    let tri = local / 3u;
    let corner = local % 3u;
    let cl = visible[slot * 2u + 1u] * 4u;
    let v_off = prim[cl];
    let t_off = prim[cl + 1u];
    let t_cnt = prim[cl + 2u];
    if (tri >= t_cnt) {
        var dead: VOut;
        dead.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0); // all three corners equal: zero-area, outside the clip volume
        return dead;
    }
    let t = t_off + tri * 3u;
    let seg = prim[t + 2u];
    if ((segstate[(inst.seg_base + seg) * 2u + 1u] & 1u) != 0u) {
        var hidden: VOut;
        hidden.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0); // a hidden object sharing this cluster
        return hidden;
    }
    let local_index = (prim[t] >> (corner * 8u)) & 0xffu;
    let vi = prim[v_off + local_index];
    return shade_vertex(inst, vi, i32(seg), ii, prim[t + 1u]);
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

// Lines wider than a pixel and points larger than one: WebGPU rasterises only 1-pixel lines and points,
// so each segment / point is a screen-space quad of 6 vertices (material.x pixels across), as three.js'
// LineSegments2 does; points are squares, like three.js' untextured PointsMaterial. A segment crossing
// the near plane is clipped there first.
@vertex
fn vs_wide(@builtin(vertex_index) vid: u32, @builtin(instance_index) ii: u32) -> VOut {
    let inst = instances[ii];
    let item = vid / 6u;
    let corner = vid % 6u;
    let along = select(0.0, 1.0, corner == 1u || corner == 2u || corner == 4u); // which end
    let side = select(-1.0, 1.0, corner == 2u || corner == 4u || corner == 5u);
    let points = (inst.flags & FLAG_POINTS) != 0u;
    let ia = select(item * 2u, item, points);
    let ib = select(item * 2u + 1u, item, points);
    let seg = prim[bitcast<u32>(inst.material.w) + ia];
    var dead: VOut;
    dead.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0);
    if ((segstate[(inst.seg_base + seg) * 2u + 1u] & 1u) != 0u) {
        return dead;
    }
    let va = shade_vertex(inst, prim[ia], i32(seg), ii, ia);
    let vb = shade_vertex(inst, prim[ib], i32(seg), ii, ia);
    var ca = va.pos;
    var cb = vb.pos;
    if (ca.z < 0.0 && cb.z < 0.0) {
        return dead;
    }
    if (ca.z < 0.0) {
        ca = mix(ca, cb, ca.z / (ca.z - cb.z));
    } else if (cb.z < 0.0) {
        cb = mix(cb, ca, cb.z / (cb.z - ca.z));
    }
    let half_px = 0.5 * max(inst.material.x, 1.0);
    let viewport = frame.params.zw;
    var out = va;
    var c = ca;
    if (along > 0.5) {
        out = vb;
        c = cb;
    }
    var offset: vec2<f32>;
    if (points) {
        offset = vec2<f32>(along * 2.0 - 1.0, side) * half_px;
    } else {
        var dir = (cb.xy / cb.w - ca.xy / ca.w) * viewport;
        if (dot(dir, dir) < 1e-12) {
            dir = vec2<f32>(1.0, 0.0);
        }
        dir = normalize(dir);
        offset = vec2<f32>(-dir.y, dir.x) * side * half_px;
    }
    // Pixels -> clip space at this vertex's depth.
    c = vec4<f32>(c.xy + offset * 2.0 / viewport * c.w, c.z, c.w);
    out.pos = c;
    return out;
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

// GPU cluster culling with two-pass occlusion culling (the "previous frame's visible set" scheme):
//   cs_early   draws clusters that were visible last frame and still pass the frustum / cone tests
//   hzb_*      build a max-depth pyramid from what the early pass drew
//   cs_late    tests every cluster against the pyramid: draws the visible ones the early pass did not,
//              and records this frame's visibility for the next one
// Anything visible in the final image passes the late test (the early depth is never nearer than the final
// depth), so nothing is lost when the camera moves; at worst a cluster is drawn late instead of early.
//
// Layout contract with renderer.cpp:
//   CullFrame  uniform, 208 bytes
//   items      one 64-byte record per (instance, cluster), in visible-region order: item i of an instance
//              with vis_base b is item b + cluster
//   history    u32 per item: drawn last frame
//   visible    (instance, cluster) pairs, see the render shader
//   args       drawIndirect args per instance (vertex count, 1, vis_base x CLUSTER_VERTS, 0)
//   counters   early clusters, early triangles, late clusters, late triangles, occluded clusters
inline constexpr const char *kCullWGSL = R"WGSL(
struct Instance {
    model: mat4x4<f32>,
    color: vec4<f32>,
    flags: u32,
    seg_base: u32,
    morph: f32,
    vis_base: u32,
    material: vec4<f32>,
};

struct CullFrame {
    view_proj: mat4x4<f32>,
    planes: array<vec4<f32>, 6>, // world space, normalised
    cam_pos: vec4<f32>,          // w: 1 = perspective
    size: vec4<f32>,             // target width, height in pixels
    params: vec4<u32>,           // pyramid levels, options (1 frustum, 2 cone, 4 occlusion), item count
};

struct Item {
    center: vec4<f32>, // xyz, radius (model space)
    axis: vec4<f32>,   // normal cone axis, cutoff
    morph_extent: f32,
    instance: u32,
    cluster: u32,
    triangles: u32,
    seg_first: u32,
    seg_last: u32,
    pad0: u32,
    pad1: u32,
};

const FLAG_TRANSPARENT: u32 = 8u;
const FLAG_HIDDEN: u32 = 16u;
const FLAG_HAS_MORPH: u32 = 64u;
const FLAG_CONE_OK: u32 = 512u; // set by the renderer: single-sided, opaque, unmorphed, similarity transform
const CLUSTER_VERTS: u32 = {{CLUSTER_VERTS}}u;

@group(0) @binding(0) var<uniform> cf: CullFrame;
@group(0) @binding(1) var<storage, read> instances: array<Instance>;
@group(0) @binding(2) var<storage, read> segstate: array<u32>;
@group(0) @binding(3) var<storage, read> items: array<Item>;
@group(0) @binding(4) var<storage, read_write> history: array<u32>;
@group(0) @binding(5) var<storage, read_write> visible: array<u32>;
@group(0) @binding(6) var<storage, read_write> args: array<atomic<u32>>;
@group(0) @binding(7) var<storage, read_write> counters: array<atomic<u32>>;
@group(0) @binding(8) var hzb: texture_2d<f32>;

// The cluster's bounding sphere in world space.
fn world_sphere(it: Item, inst: Instance) -> vec4<f32> {
    let m = inst.model;
    let scale = max(length(m[0].xyz), max(length(m[1].xyz), length(m[2].xyz)));
    var r = it.center.w;
    if ((inst.flags & FLAG_HAS_MORPH) != 0u) {
        r = r + abs(inst.morph) * it.morph_extent;
    }
    return vec4<f32>((m * vec4<f32>(it.center.xyz, 1.0)).xyz, r * scale);
}

// Hidden, outside the frustum or facing away: the tests that need no depth.
fn passes(it: Item, inst: Instance, s: vec4<f32>) -> bool {
    if ((inst.flags & FLAG_HIDDEN) != 0u) {
        return false;
    }
    var any_visible = false;
    for (var sg = it.seg_first; sg <= it.seg_last; sg = sg + 1u) {
        if ((segstate[(inst.seg_base + sg) * 2u + 1u] & 1u) == 0u) {
            any_visible = true;
            break;
        }
    }
    if (!any_visible) {
        return false;
    }
    if ((cf.params.y & 1u) != 0u) {
        for (var p = 0u; p < 6u; p = p + 1u) {
            if (dot(cf.planes[p].xyz, s.xyz) + cf.planes[p].w < -s.w) {
                return false;
            }
        }
    }
    if ((cf.params.y & 2u) != 0u && (inst.flags & FLAG_CONE_OK) != 0u && cf.cam_pos.w > 0.5) {
        let axis = normalize((inst.model * vec4<f32>(it.axis.xyz, 0.0)).xyz);
        let d = s.xyz - cf.cam_pos.xyz;
        if (dot(d, axis) >= it.axis.w * length(d) + s.w) {
            return false;
        }
    }
    return true;
}

// Behind what the pyramid holds: the nearest depth of the sphere's box is farther than the farthest depth
// over the screen rectangle it covers. Conservative: the box bounds the sphere, the rectangle the box.
fn occluded(s: vec4<f32>) -> bool {
    let c = cf.view_proj * vec4<f32>(s.xyz, 1.0);
    let ex = cf.view_proj[0] * s.w;
    let ey = cf.view_proj[1] * s.w;
    let ez = cf.view_proj[2] * s.w;
    var lo = vec3<f32>(1e30);
    var hi = vec3<f32>(-1e30);
    for (var k = 0u; k < 8u; k = k + 1u) {
        let p = c + select(-ex, ex, (k & 1u) != 0u) + select(-ey, ey, (k & 2u) != 0u) +
                select(-ez, ez, (k & 4u) != 0u);
        if (p.w <= 1e-6) {
            return false; // reaches behind the camera
        }
        let n = p.xyz / p.w;
        lo = min(lo, n);
        hi = max(hi, n);
    }
    if (lo.z <= 0.0) {
        return false; // crosses the near plane
    }
    let size = cf.size.xy;
    let x0 = u32(clamp((lo.x * 0.5 + 0.5) * size.x, 0.0, size.x - 1.0));
    let x1 = u32(clamp((hi.x * 0.5 + 0.5) * size.x, 0.0, size.x - 1.0));
    let y0 = u32(clamp((0.5 - hi.y * 0.5) * size.y, 0.0, size.y - 1.0));
    let y1 = u32(clamp((0.5 - lo.y * 0.5) * size.y, 0.0, size.y - 1.0));
    // The level where the rectangle spans at most 2 x 2 texels.
    let extent = f32(max(x1 - x0, y1 - y0) + 1u);
    let level = min(u32(ceil(log2(extent))), cf.params.x - 1u);
    let dims = textureDimensions(hzb, level) - vec2<u32>(1u);
    let a = min(vec2<u32>(x0, y0) >> vec2<u32>(level), dims);
    let b = min(vec2<u32>(x1, y1) >> vec2<u32>(level), dims);
    let far = max(max(textureLoad(hzb, a, level).r, textureLoad(hzb, vec2<u32>(b.x, a.y), level).r),
                  max(textureLoad(hzb, vec2<u32>(a.x, b.y), level).r, textureLoad(hzb, b, level).r));
    return lo.z > far;
}

fn emit(it: Item, inst: Instance, counter: u32) {
    let n = atomicAdd(&args[it.instance * 4u], CLUSTER_VERTS) / CLUSTER_VERTS;
    let o = (inst.vis_base + n) * 2u;
    visible[o] = it.instance;
    visible[o + 1u] = it.cluster;
    atomicAdd(&counters[counter], 1u);
    atomicAdd(&counters[counter + 1u], it.triangles);
}

// Items beyond 65535 workgroups spill into the dispatch's second dimension.
fn item_index(gid: vec3<u32>, groups: vec3<u32>) -> u32 {
    return gid.x + gid.y * groups.x * 64u;
}

@compute @workgroup_size(64)
fn cs_early(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) groups: vec3<u32>) {
    let i = item_index(gid, groups);
    if (i >= cf.params.z || history[i] == 0u) {
        return;
    }
    let it = items[i];
    let inst = instances[it.instance];
    if ((inst.flags & FLAG_TRANSPARENT) != 0u || !passes(it, inst, world_sphere(it, inst))) {
        return;
    }
    emit(it, inst, 0u);
}

@compute @workgroup_size(64)
fn cs_late(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) groups: vec3<u32>) {
    let i = item_index(gid, groups);
    if (i >= cf.params.z) {
        return;
    }
    let it = items[i];
    let inst = instances[it.instance];
    let s = world_sphere(it, inst);
    let drawn_last = history[i] != 0u;
    if (!passes(it, inst, s)) {
        history[i] = 0u;
        return;
    }
    if ((cf.params.y & 4u) != 0u && occluded(s)) {
        history[i] = 0u;
        atomicAdd(&counters[4], 1u);
        return;
    }
    let transparent = (inst.flags & FLAG_TRANSPARENT) != 0u;
    history[i] = select(1u, 0u, transparent); // transparent clusters never occlude, so never draw early
    if (drawn_last && !transparent) {
        return; // the early pass drew it
    }
    emit(it, inst, 2u);
}

// Depth pyramid. Level 0: the farthest sample of each pixel of the multisampled depth buffer.
@group(0) @binding(0) var depth_ms: texture_depth_multisampled_2d;
@group(0) @binding(1) var hzb_out0: texture_storage_2d<r32float, write>;

@compute @workgroup_size(8, 8)
fn hzb_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(hzb_out0);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    var d = 0.0;
    for (var s = 0u; s < textureNumSamples(depth_ms); s = s + 1u) {
        d = max(d, textureLoad(depth_ms, gid.xy, s));
    }
    textureStore(hzb_out0, gid.xy, vec4<f32>(d, 0.0, 0.0, 0.0));
}

// Level n from level n - 1: the farthest of each 2 x 2 block. Mip sizes round down, so the last texel of
// a row / column also takes the finer level's odd last row / column; the coarser level then still covers
// every pixel (a pixel p at level n lies in texel min(p >> n, size - 1)).
@group(0) @binding(0) var hzb_src: texture_2d<f32>;
@group(0) @binding(1) var hzb_dst: texture_storage_2d<r32float, write>;

@compute @workgroup_size(8, 8)
fn hzb_reduce(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(hzb_dst);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let src = textureDimensions(hzb_src);
    let last = src - vec2<u32>(1u);
    let odd = (gid.xy == size - vec2<u32>(1u)) & ((src & vec2<u32>(1u)) == vec2<u32>(1u));
    let span = select(vec2<u32>(2u), vec2<u32>(3u), odd);
    var d = 0.0;
    for (var y = 0u; y < span.y; y = y + 1u) {
        for (var x = 0u; x < span.x; x = x + 1u) {
            d = max(d, textureLoad(hzb_src, min(gid.xy * 2u + vec2<u32>(x, y), last), 0).r);
        }
    }
    textureStore(hzb_dst, gid.xy, vec4<f32>(d, 0.0, 0.0, 0.0));
}
)WGSL";

} // namespace adacpp::render
