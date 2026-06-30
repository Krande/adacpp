// embind entry point for the in-browser GLB diff (mirrors cad_wasm.cpp's no-pyodide step→glb module).
// Reuses the portable diff core (summarize_glb_buf + diff_summaries + write_overlay_glb) from
// glb_diff_native.h — no tinygltf, just nlohmann json + meshoptimizer, both already in the wasm build.
// The disk-spill path is #ifndef __EMSCRIPTEN__'d out, so wasm decodes meshopt bufferViews in RAM.
//
// JS: const m = await createAdacppGlbDiff();
//     const { ops, counts, overlay } = m.diffGlb(sceneU8, refU8, "byName", 1e-3, 0xD50000FF);
//     // ops: [[node_id, status], ...]  status 0=unchanged 1=added 2=removed 3=modified
//     // counts: {added, removed, modified, unchanged};  overlay: Uint8Array (red ref-only GLB)

#include <string>
#include <unordered_set>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "../diff/glb_diff_native.h"

using namespace adacpp::gdiff;

namespace {

Mode parse_mode(const std::string &s) {
    if (s == "byName")
        return Mode::ByName;
    if (s == "byGuid")
        return Mode::ByGuid;
    if (s == "byCentroid")
        return Mode::ByCentroid;
    if (s == "byProperty")
        return Mode::ByProperty;
    return Mode::NameThenCentroid;
}

emscripten::val to_u8(const std::string &bytes) {
    return emscripten::val::global("Uint8Array")
        .new_(emscripten::typed_memory_view(bytes.size(), (const unsigned char *) bytes.data()));
}

// Diff two GLBs (Uint8Array each). Returns {ops, counts, overlay}. The scene GLB is summarised once,
// the ref twice (summary + the removed-overlay re-scan) — geometry is never fully held: one node's
// decode at a time, exactly like the native path (sans mmap; the input buffer is already in heap).
emscripten::val diff_glb(emscripten::val scene_arr, emscripten::val ref_arr, const std::string &mode_s, double tol,
                         unsigned int overlay_rgba) {
    std::vector<unsigned char> scene = emscripten::convertJSArrayToNumberVector<unsigned char>(scene_arr);
    std::vector<unsigned char> ref = emscripten::convertJSArrayToNumberVector<unsigned char>(ref_arr);
    Mode mode = parse_mode(mode_s);

    std::vector<ElementSummary> ss = summarize_glb_buf(scene.data(), scene.size());
    std::vector<ElementSummary> rs = summarize_glb_buf(ref.data(), ref.size());
    DiffResult res = diff_summaries(ss, rs, mode, tol);

    std::string overlay;
    if (!res.removed_node_ids.empty()) {
        std::unordered_set<std::string> keep(res.removed_node_ids.begin(), res.removed_node_ids.end());
        std::vector<float> tris;
        summarize_glb_buf(ref.data(), ref.size(), &keep, &tris);
        overlay = write_overlay_glb(tris, overlay_rgba);
    }

    emscripten::val out = emscripten::val::object();
    emscripten::val ops = emscripten::val::array();
    for (const DiffOp &o : res.ops) {
        emscripten::val op = emscripten::val::array();
        op.call<void>("push", o.node_id);
        op.call<void>("push", (int) o.status);
        ops.call<void>("push", op);
    }
    out.set("ops", ops);
    emscripten::val counts = emscripten::val::object();
    counts.set("added", res.n_added);
    counts.set("removed", res.n_removed);
    counts.set("modified", res.n_modified);
    counts.set("unchanged", res.n_unchanged);
    out.set("counts", counts);
    out.set("overlay", to_u8(overlay));
    return out;
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_glb_diff) {
    emscripten::function("diffGlb", &diff_glb);
}
