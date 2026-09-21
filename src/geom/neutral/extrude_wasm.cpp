// embind entry point for prismatic extrusion expansion (mirrors glb_diff_wasm.cpp's shape).
//
// This is the smallest module in the set on purpose: the expander in ngeom_extrude.h is
// header-only, so nothing here links libtess2, meshoptimizer, manifold or OCCT. The section tables
// it replays are prepared by whatever wrote the artefact; the browser only ever sweeps them.
//
// It exists so the expansion arithmetic has ONE implementation. A viewer that expands compact
// instance records in hand-written JavaScript, and a writer that expands them in its own language
// to produce the reference, are two implementations of one formula held together by a fixture —
// and they drift. Both call this instead.
//
// JS: const m = await createAdacppExtrude();
//     const out = m.expandBeamSolids(
//         [{points: Float64Array(n*2), triangles: Uint32Array(m*3)}, ...],   // section table
//         {label, section, node0, node1,      // Uint32Array, one per instance
//          origin, xvec, yvec,                // Float64Array, 3 per instance
//          length},                           // Float64Array, one per instance
//         points);                            // Float64Array, 3 per point
//     // out: {positions: Float32Array, indices: Uint32Array, node0, node1: Uint32Array,
//     //       t: Float32Array, rangeLabel, rangeTriStart, rangeTriCount: Uint32Array}
#include <cstdint>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "ngeom_extrude.h"

using namespace adacpp::ngeom;

namespace {

// Construct the JS typed array from a view of our heap. `new Uint32Array(view)` copies once, which
// is what keeps the result valid after these vectors go out of scope — handing back a bare view
// would dangle the moment this call returns.
template <typename T> emscripten::val to_typed(const char *ctor, const std::vector<T> &v) {
    return emscripten::val::global(ctor).new_(emscripten::typed_memory_view(v.size(), v.data()));
}

std::vector<double> f64(const emscripten::val &v) {
    return emscripten::convertJSArrayToNumberVector<double>(v);
}
std::vector<uint32_t> u32(const emscripten::val &v) {
    return emscripten::convertJSArrayToNumberVector<uint32_t>(v);
}

emscripten::val expand(emscripten::val sections_val, emscripten::val instances_val, emscripten::val points_val) {
    std::vector<ExtrudedSection> sections;
    const size_t n_sec = sections_val["length"].as<size_t>();
    sections.reserve(n_sec);
    for (size_t s = 0; s < n_sec; ++s) {
        emscripten::val sv = sections_val[s];
        std::vector<double> pts = f64(sv["points"]);
        std::vector<uint32_t> tris = u32(sv["triangles"]);
        ExtrudedSection sec;
        sec.points.resize(pts.size() / 2);
        for (size_t i = 0; i < sec.points.size(); ++i)
            sec.points[i] = {pts[i * 2], pts[i * 2 + 1]};
        sec.triangles.resize(tris.size() / 3);
        for (size_t i = 0; i < sec.triangles.size(); ++i)
            sec.triangles[i] = {tris[i * 3], tris[i * 3 + 1], tris[i * 3 + 2]};
        sec.ok = true;
        sections.push_back(std::move(sec));
    }

    const std::vector<uint32_t> label = u32(instances_val["label"]);
    const std::vector<uint32_t> section = u32(instances_val["section"]);
    const std::vector<uint32_t> node0 = u32(instances_val["node0"]);
    const std::vector<uint32_t> node1 = u32(instances_val["node1"]);
    const std::vector<double> origin = f64(instances_val["origin"]);
    const std::vector<double> xvec = f64(instances_val["xvec"]);
    const std::vector<double> yvec = f64(instances_val["yvec"]);
    const std::vector<double> length = f64(instances_val["length"]);

    const size_t n = label.size();
    std::vector<BeamInstance> beams(n);
    for (size_t k = 0; k < n; ++k) {
        BeamInstance &b = beams[k];
        b.label = label[k];
        b.section = k < section.size() ? section[k] : 0;
        b.node0 = k < node0.size() ? node0[k] : 0;
        b.node1 = k < node1.size() ? node1[k] : 0;
        b.length = k < length.size() ? length[k] : 0.0;
        for (size_t c = 0; c < 3; ++c) {
            b.origin[c] = k * 3 + c < origin.size() ? origin[k * 3 + c] : 0.0;
            b.xvec[c] = k * 3 + c < xvec.size() ? xvec[k * 3 + c] : 0.0;
            b.yvec[c] = k * 3 + c < yvec.size() ? yvec[k * 3 + c] : 0.0;
        }
    }

    BeamSolidMesh mesh = expand_beam_solids(sections, beams, f64(points_val));

    std::vector<uint32_t> r_label, r_start, r_count;
    r_label.reserve(mesh.ranges.size());
    r_start.reserve(mesh.ranges.size());
    r_count.reserve(mesh.ranges.size());
    for (const InstanceRange &r : mesh.ranges) {
        r_label.push_back(r.label);
        r_start.push_back(r.tri_start);
        r_count.push_back(r.tri_count);
    }

    emscripten::val out = emscripten::val::object();
    out.set("positions", to_typed("Float32Array", mesh.positions));
    out.set("indices", to_typed("Uint32Array", mesh.indices));
    out.set("node0", to_typed("Uint32Array", mesh.node0));
    out.set("node1", to_typed("Uint32Array", mesh.node1));
    out.set("t", to_typed("Float32Array", mesh.t));
    out.set("rangeLabel", to_typed("Uint32Array", r_label));
    out.set("rangeTriStart", to_typed("Uint32Array", r_start));
    out.set("rangeTriCount", to_typed("Uint32Array", r_count));
    return out;
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_extrude) {
    emscripten::function("expandBeamSolids", &expand);
}
