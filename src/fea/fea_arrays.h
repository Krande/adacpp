// Format-neutral result arrays: the shapes, constants and per-step statistics shared by the FEA kernels
// (superpose / derive / envelope) and the artefact writer.
//
// A "stride" is one result step of one field: a dense row-major float32 block of `rows x n_components`,
// where `rows` is `n_points` for a nodal field (AFBL) and `n_elements * n_ips` for an element field
// (AFEL). Every kernel in src/fea operates on strides and nothing else, so the same code serves the
// server (nanobind, files on disk) and the browser (embind, files in OPFS).
//
// NUMERICS CONTRACT. These kernels must give the same BITS on every engine (native x86-64/arm64, wasm
// with and without SIMD). They are compiled with -ffp-contract=off (no fused multiply-add), never with
// -ffast-math, and never with relaxed-simd; every operation is a single correctly-rounded IEEE-754 op
// in a fixed order. See superpose.h and derive.h for the order each kernel commits to.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace adacpp::fea {

// Binary sidecar constants. Byte-identical to adapy's ada/fem/results/artefacts/formats.py.
inline constexpr char AFBL_MAGIC[4] = {'A', 'F', 'B', 'L'};
inline constexpr char AFEL_MAGIC[4] = {'A', 'F', 'E', 'L'};
inline constexpr uint32_t AFBL_VERSION = 1;
inline constexpr uint32_t AFEL_VERSION = 1;
inline constexpr std::size_t BLOB_HEADER_BYTES = 1024;
// Envelope governing-case sidecar (adapy artefacts/combine.py write_envelope): a 16-byte header --
// "AFGV", uint32 version 1, uint32 n_cases, uint32 0 -- then uint16 [2 x rows x n_components]
// (row 0 = the case that set the max, row 1 = the min), little-endian.
inline constexpr char AFGV_MAGIC[4] = {'A', 'F', 'G', 'V'};
inline constexpr uint32_t AFGV_VERSION = 1;
inline constexpr std::size_t AFGV_HEADER_BYTES = 16;

// Per-step value ranges, exactly as adapy's FieldBlobWriter / ElementFieldBlobWriter range one step:
//   - per component: min / max over the FINITE values of that column (NaN/inf skipped);
//   - magnitude (only when n_components >= 3): sqrt((x0*x0 + x1*x1) + x2*x2) evaluated in float32 --
//     np.linalg.norm(arr[..., :3], axis=-1) on a float32 array -- min / max over the finite results.
// `has_*` is false when a column (or the magnitude) had no finite value; the manifest then records
// (0, 0), which `range_or_zero` reproduces.
struct StepStats {
    std::size_t rows = 0;
    std::size_t n_components = 0;
    std::vector<float> comp_min, comp_max;
    std::vector<uint8_t> comp_has;
    bool has_magnitude = false; // n_components >= 3
    bool magnitude_finite = false;
    float mag_min = 0.0f, mag_max = 0.0f;
};

StepStats compute_step_stats(const float *values, std::size_t rows, std::size_t n_components);

// Fold `other` into `acc` (ranges over several steps; used for the two-step envelope blob).
void merge_step_stats(StepStats &acc, const StepStats &other);

// Shortest round-trip decimal for a double (std::to_chars). JSON-safe: non-finite values print as
// null. Shared by the WASM stats JSON so both engines report the same text.
std::string format_double(double v);

// `{"rows":..,"n_components":..,"scalar_range_per_component":[[min,max],...],"scalar_range_magnitude":[min,max]}`
std::string step_stats_json(const StepStats &s);

} // namespace adacpp::fea
