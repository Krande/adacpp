// embind entry for the FEA kernels wasm module (adacpp_fea): load combinations, derived components and
// envelopes over baked AFBL/AFEL strides, in the browser.
//
// STANDALONE wasm module, like the other adacpp_* embind modules: no OCCT, no pyodide, no Python.
// Every verb calls the SAME C++ the nanobind module exposes (field_ops.h), so a case materialised here
// and one materialised on the server are the same bytes.
//
// IO: all paths live in the emscripten FS (WASMFS). In a Web Worker, `await Module.opfsMount("/opfs")`
// and `await Module.opfsOpen(path)` the blobs (src/wasmio/opfs_sync.js), then pass paths under the
// mount: strides are read one at a time with pread (bounded RSS, whatever the blob size) and the result
// is written straight back to OPFS. Single-threaded (no SharedArrayBuffer needed).
//
// Errors never throw into JS: every verb returns a JSON string, {"ok":true,...} or
// {"ok":false,"error":"..."}.
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <nlohmann/json.hpp>

#include "artefact_io.h"
#include "derive.h"
#include "field_ops.h"

using adacpp::fea::CombineOptions;
using adacpp::fea::DeriveSpec;
using adacpp::fea::StrideRef;
using json = nlohmann::json;

namespace {

std::string error_json(const std::string &what) {
    return json{{"ok", false}, {"error", what}}.dump();
}

// inPathsJson: a JSON array of paths, one per term. stepIdx: a number (same step for every term) or
// an array / typed array with one step per term.
std::vector<StrideRef> parse_terms(const std::string &in_paths_json, const emscripten::val &step_idx) {
    const json paths = json::parse(in_paths_json);
    if (!paths.is_array() || paths.empty())
        throw std::invalid_argument("inPathsJson must be a non-empty JSON array of paths");
    std::vector<StrideRef> terms;
    terms.reserve(paths.size());
    for (const auto &p : paths)
        terms.push_back({p.get<std::string>(), 0});

    if (step_idx.isNumber()) {
        const double s = step_idx.as<double>();
        if (s < 0 || std::floor(s) != s)
            throw std::invalid_argument("stepIdx must be a non-negative integer");
        for (auto &t : terms)
            t.step = static_cast<uint64_t>(s);
    } else {
        const std::vector<double> steps = emscripten::vecFromJSArray<double>(step_idx);
        if (steps.size() != terms.size())
            throw std::invalid_argument("stepIdx: one step per path");
        for (std::size_t i = 0; i < steps.size(); ++i) {
            if (steps[i] < 0 || std::floor(steps[i]) != steps[i])
                throw std::invalid_argument("stepIdx must hold non-negative integers");
            terms[i].step = static_cast<uint64_t>(steps[i]);
        }
    }
    return terms;
}

// deriveJson: "" (no derivation), a JSON array of ops, or
//   {"ops":[{"op":"plane_von_mises","args":[0,1,2],"out":[3]}, ...], "n_components":2, "name":"P-STRESS"}
CombineOptions parse_derive(const std::string &derive_json) {
    CombineOptions opts;
    if (derive_json.empty())
        return opts;
    const json d = json::parse(derive_json);
    const json *ops = &d;
    if (d.is_object()) {
        if (d.contains("n_components"))
            opts.n_components = d.at("n_components").get<uint64_t>();
        if (d.contains("name"))
            opts.name = d.at("name").get<std::string>();
        if (!d.contains("ops"))
            return opts;
        ops = &d.at("ops");
    }
    if (!ops->is_array())
        throw std::invalid_argument("deriveJson: ops must be an array");
    for (const auto &o : *ops) {
        DeriveSpec s;
        s.op = adacpp::fea::derive_op_from_name(o.at("op").get<std::string>());
        s.args = o.at("args").get<std::vector<int>>();
        s.out = o.at("out").get<std::vector<int>>();
        opts.derive.push_back(std::move(s));
    }
    return opts;
}

std::string combine_field(const std::string &in_paths_json, emscripten::val step_idx, emscripten::val factors,
                          const std::string &out_path, const std::string &derive_json) {
    try {
        const std::vector<StrideRef> terms = parse_terms(in_paths_json, step_idx);
        // A Float32Array arrives exactly; plain JS numbers (doubles) are rounded to float32 here, once,
        // to nearest -- the same rounding np.float32(x) applies.
        const std::vector<double> fd = emscripten::vecFromJSArray<double>(factors);
        std::vector<float> f(fd.begin(), fd.end());
        const CombineOptions opts = parse_derive(derive_json);
        return adacpp::fea::field_op_result_json(adacpp::fea::combine_field_files(terms, f, out_path, opts));
    } catch (const std::exception &e) {
        return error_json(e.what());
    }
}

std::string envelope_field(const std::string &in_paths_json, emscripten::val step_idx, const std::string &out_path,
                           const std::string &gov_path) {
    try {
        const std::vector<StrideRef> cases = parse_terms(in_paths_json, step_idx);
        return adacpp::fea::field_op_result_json(adacpp::fea::envelope_field_files(cases, out_path, gov_path));
    } catch (const std::exception &e) {
        return error_json(e.what());
    }
}

// The blob header as JSON (+ "kind"), or {"ok":false,...}.
std::string read_blob_header(const std::string &path) {
    try {
        const adacpp::fea::BlobHeader h = adacpp::fea::read_header(path);
        json j = json::parse(adacpp::fea::header_json(h));
        j["kind"] = h.kind == adacpp::fea::BlobKind::AFEL ? "AFEL" : "AFBL";
        j["ok"] = true;
        return j.dump();
    } catch (const std::exception &e) {
        return error_json(e.what());
    }
}

std::string version() {
    return "adacpp_fea/1";
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_fea) {
    emscripten::function("combineField", &combine_field);
    emscripten::function("envelopeField", &envelope_field);
    emscripten::function("readBlobHeader", &read_blob_header);
    emscripten::function("version", &version);
}
