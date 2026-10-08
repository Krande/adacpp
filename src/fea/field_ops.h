// File-level FEA operations: the one implementation behind both engines.
//
// The browser (fea_wasm.cpp, files in OPFS via WASMFS) and the server (fea_py_wrap.cpp, files on disk)
// call exactly these functions, so a case materialised in either place is the same file, byte for byte.
// Strides stream one at a time (two stride buffers live at once, whatever the term count), so memory
// is bounded by the field size, not by the number of terms.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "artefact_io.h"
#include "derive.h"
#include "fea_arrays.h"

namespace adacpp::fea {

// One stored-case stride: step `step` of the AFBL/AFEL blob at `path`.
struct StrideRef {
    std::string path;
    uint64_t step = 0;
};

struct CombineOptions {
    // Derivation ops, applied in order after superposition (derive.h). With `n_components == 0` they
    // run IN PLACE on the combined stride (args and outputs index the field's own columns). With
    // `n_components > 0` the output is a NEW layout of that many columns, initialised to NaN: args
    // index the combined input, outputs index the new layout (use `copy` for passthrough columns).
    std::vector<DeriveSpec> derive;
    uint64_t n_components = 0;
    std::string name; // output field name; empty keeps the input's
};

struct FieldOpResult {
    BlobHeader header;            // what was written
    std::vector<StepStats> stats; // one per written step
    double ms_read = 0, ms_combine = 0, ms_derive = 0, ms_write = 0;
};

// Materialise one load combination: out = sum_t factors[t] * stride(terms[t]) (superpose.h order),
// derive, and write a single-step (n_steps = 1) AFBL/AFEL blob with the input's header kind, name and
// element type. Every term must have the same kind / shape / element type. Throws on any mismatch.
FieldOpResult combine_field_files(const std::vector<StrideRef> &terms, const std::vector<float> &factors,
                                  const std::string &out_path, const CombineOptions &opts = {});

// Envelope over cases (envelope.h): writes a TWO-step blob at `out_path` (step 0 = max, step 1 = min;
// same header as the inputs otherwise, so the viewer's per-step range read applies unchanged) and, when
// `gov_path` is not empty, the governing case indices as an AFGV sidecar (fea_arrays.h: 16-byte header,
// then uint16 [2 x rows x n_comp], same step order) -- byte-identical to adapy's write_envelope.
FieldOpResult envelope_field_files(const std::vector<StrideRef> &cases, const std::string &out_path,
                                   const std::string &gov_path);

// Stats JSON for a result: {"ok":true,"name":..,"n_steps":..,"rows":..,"n_components":..,
// "steps":[<step_stats_json>...],"timing_ms":{...}}
std::string field_op_result_json(const FieldOpResult &r);

} // namespace adacpp::fea
