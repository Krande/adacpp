// Python bindings for the format-neutral FEA kernels (src/fea).
//
// Two layers, both thin:
//   - array kernels on numpy float32 arrays (combine_strides / derive / envelope / step_stats), for a
//     caller that already holds strides in memory (adapy's numpy pipeline swaps these in);
//   - the file verbs (combine_field / envelope_field / write_afbl / write_afel / read_header), which
//     call the very functions the wasm module calls (field_ops.h), so a case the server materialises
//     is byte-identical to one a browser materialises.
// The GIL is released around every kernel.
#include "fea_py_wrap.h"

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>

#include "artefact_io.h"
#include "derive.h"
#include "envelope.h"
#include "fea_arrays.h"
#include "field_ops.h"
#include "superpose.h"

using namespace adacpp::fea;

namespace {

using F32In = nb::ndarray<const float, nb::c_contig, nb::device::cpu>;
using F32Out = nb::ndarray<float, nb::c_contig, nb::device::cpu>;
using F64In = nb::ndarray<const double, nb::c_contig, nb::device::cpu>;

template <typename T> nb::ndarray<nb::numpy, T> new_array(const std::vector<std::size_t> &shape) {
    std::size_t n = 1;
    for (std::size_t s : shape)
        n *= s;
    T *data = new T[n > 0 ? n : 1];
    nb::capsule owner(data, [](void *p) noexcept { delete[] static_cast<T *>(p); });
    return nb::ndarray<nb::numpy, T>(data, shape.size(), shape.data(), owner);
}

std::vector<std::size_t> shape_of(const F32In &a) {
    std::vector<std::size_t> s(a.ndim());
    for (std::size_t i = 0; i < a.ndim(); ++i)
        s[i] = a.shape(i);
    return s;
}

std::vector<float> to_f32(const std::vector<double> &v) {
    // Python floats are doubles: round each to float32 once, to nearest (np.float32(x)).
    return std::vector<float>(v.begin(), v.end());
}

nb::object json_loads(const std::string &s) {
    return nb::module_::import_("json").attr("loads")(s);
}

DeriveSpec spec_from_dict(const nb::dict &d) {
    DeriveSpec s;
    s.op = derive_op_from_name(nb::cast<std::string>(d["op"]));
    s.args = nb::cast<std::vector<int>>(d["args"]);
    s.out = nb::cast<std::vector<int>>(d["out"]);
    return s;
}

std::vector<StrideRef> stride_refs(const std::vector<std::string> &paths, const std::vector<uint64_t> &steps) {
    if (steps.size() != paths.size() && steps.size() != 1)
        throw std::invalid_argument("steps: one per path, or a single step for every path");
    std::vector<StrideRef> refs;
    for (std::size_t i = 0; i < paths.size(); ++i)
        refs.push_back({paths[i], steps.size() == 1 ? steps[0] : steps[i]});
    return refs;
}

} // namespace

void fea_module(nb::module_ &m) {
    m.doc() = "Format-neutral FEA result kernels: load-combination superposition, derived components, "
              "envelopes and the AFBL/AFEL artefact writer. Bit-identical to the adacpp_fea wasm module.";

    m.def(
        "combine_strides",
        [](const std::vector<F32In> &inputs, const std::vector<double> &factors) {
            if (inputs.empty())
                throw std::invalid_argument("combine_strides: need at least one input");
            if (inputs.size() != factors.size())
                throw std::invalid_argument("combine_strides: one factor per input");
            const std::size_t n = inputs[0].size();
            std::vector<const float *> ptrs;
            for (const auto &a : inputs) {
                if (a.size() != n)
                    throw std::invalid_argument("combine_strides: inputs differ in size");
                ptrs.push_back(a.data());
            }
            const std::vector<float> c = to_f32(factors);
            auto out = new_array<float>(shape_of(inputs[0]));
            {
                nb::gil_scoped_release release;
                combine_strides(ptrs, c, out.data(), n);
            }
            return out;
        },
        "inputs"_a, "factors"_a,
        "sum(factors[t] * inputs[t]) in float32: out = c0*x0, then out += ct*xt in the given order, each\n"
        "product and sum rounded to float32 (no FMA). Factors are rounded to float32 once. Returns a new\n"
        "array shaped like inputs[0]; bit-identical to numpy's `out = c0*x0; out += c1*x1; ...`.");

    m.def(
        "derive",
        [](const std::string &op, F32Out values, const std::vector<int> &args, const std::vector<int> &out_cols,
           std::optional<F32Out> out, std::optional<F64In> thickness, std::size_t thickness_rows) {
            if (values.ndim() < 1)
                throw std::invalid_argument("derive: values must be (..., n_components)");
            const std::size_t in_nc = values.shape(values.ndim() - 1);
            const std::size_t rows = in_nc ? values.size() / in_nc : 0;
            DeriveSpec spec{derive_op_from_name(op), args, out_cols};
            float *dst = values.data();
            std::size_t out_nc = in_nc;
            if (out) {
                out_nc = out->shape(out->ndim() - 1);
                if (out_nc == 0 || out->size() / out_nc != rows)
                    throw std::invalid_argument("derive: out must have the same rows as values");
                dst = out->data();
            }
            const double *th = nullptr;
            if (thickness) {
                if (thickness_rows == 0 || thickness->size() * thickness_rows < rows)
                    throw std::invalid_argument("derive: thickness too short for the rows");
                th = thickness->data();
            }
            validate_derive(spec, in_nc, out_nc, th != nullptr);
            {
                nb::gil_scoped_release release;
                apply_derive(spec, values.data(), in_nc, dst, out_nc, rows, th, thickness_rows);
            }
        },
        "op"_a, "values"_a.noconvert(), "args"_a, "out_cols"_a, "out"_a.noconvert() = nb::none(),
        "thickness"_a = nb::none(), "thickness_rows"_a = 1,
        "Apply one derivation op over a stride (..., n_components) float32, in place (out=None) or into\n"
        "`out` (..., n_out). `args` / `out_cols` are column indices; -1 in out_cols drops that output.\n"
        "Ops: copy, plane_von_mises, plane_principal, magnitude (1..7 args), shell_decompose (bottom[3],\n"
        "top[3] -> 7), shell_resultants (6 D-STRESS comps + per-row float64 thickness, one value per\n"
        "`thickness_rows` rows). Same operation order as adapy's derived_values.py, in double, rounded\n"
        "to float32 once.");

    m.def(
        "envelope",
        [](const std::vector<F32In> &inputs) {
            if (inputs.empty())
                throw std::invalid_argument("envelope: need at least one input");
            const std::size_t n = inputs[0].size();
            std::vector<const float *> ptrs;
            for (const auto &a : inputs) {
                if (a.size() != n)
                    throw std::invalid_argument("envelope: inputs differ in size");
                ptrs.push_back(a.data());
            }
            const auto shape = shape_of(inputs[0]);
            auto mx = new_array<float>(shape);
            auto mn = new_array<float>(shape);
            auto gx = new_array<uint16_t>(shape);
            auto gn = new_array<uint16_t>(shape);
            {
                nb::gil_scoped_release release;
                envelope_strides(ptrs, n, mx.data(), mn.data(), gx.data(), gn.data());
            }
            return nb::make_tuple(mx, mn, gx, gn);
        },
        "inputs"_a,
        "Element-wise (max, min, governing_max, governing_min) over the inputs (cases, in order).\n"
        "NaN is skipped; ties keep the earlier case; governing indices are uint16.");

    m.def(
        "step_stats",
        [](const F32In &values) {
            if (values.ndim() < 1)
                throw std::invalid_argument("step_stats: values must be (..., n_components)");
            const std::size_t nc = values.shape(values.ndim() - 1);
            const std::size_t rows = nc ? values.size() / nc : 0;
            StepStats s;
            {
                nb::gil_scoped_release release;
                s = compute_step_stats(values.data(), rows, nc);
            }
            return json_loads(step_stats_json(s));
        },
        "values"_a,
        "Per-step ranges as the artefact writers record them: finite min/max per component, and the\n"
        "float32 magnitude of the first three components (0, 0 when absent).");

    m.def(
        "encode_afbl_header",
        [](const std::string &name, uint64_t n_steps, uint64_t n_points, uint64_t n_components) {
            const auto b = encode_header(make_afbl_header(name, n_steps, n_points, n_components));
            return nb::bytes(reinterpret_cast<const char *>(b.data()), b.size());
        },
        "name"_a, "n_steps"_a, "n_points"_a, "n_components"_a, "The exact 1024-byte AFBL prefix.");

    m.def(
        "encode_afel_header",
        [](const std::string &name, const std::string &elem_type, uint64_t n_steps, uint64_t n_elements, uint64_t n_ips,
           uint64_t n_components) {
            const auto b = encode_header(make_afel_header(name, elem_type, n_steps, n_elements, n_ips, n_components));
            return nb::bytes(reinterpret_cast<const char *>(b.data()), b.size());
        },
        "name"_a, "elem_type"_a, "n_steps"_a, "n_elements"_a, "n_ips"_a, "n_components"_a,
        "The exact 1024-byte AFEL prefix.");

    m.def(
        "write_afbl",
        [](const std::string &path, const std::string &name, const F32In &data) {
            if (data.ndim() != 3)
                throw std::invalid_argument("write_afbl: data must be (n_steps, n_points, n_components)");
            nb::gil_scoped_release release;
            write_afbl(path, name, data.data(), data.shape(0), data.shape(1), data.shape(2));
        },
        "path"_a, "name"_a, "data"_a, "Write a whole AFBL blob from (n_steps, n_points, n_components) float32.");

    m.def(
        "write_afel",
        [](const std::string &path, const std::string &name, const std::string &elem_type, const F32In &data) {
            if (data.ndim() != 4)
                throw std::invalid_argument("write_afel: data must be (n_steps, n_elements, n_ips, n_components)");
            nb::gil_scoped_release release;
            write_afel(path, name, elem_type, data.data(), data.shape(0), data.shape(1), data.shape(2), data.shape(3));
        },
        "path"_a, "name"_a, "elem_type"_a, "data"_a,
        "Write a whole AFEL blob from (n_steps, n_elements, n_ips, n_components) float32.");

    m.def(
        "read_header",
        [](const std::string &path) {
            const BlobHeader h = read_header(path);
            nb::object d = json_loads(header_json(h));
            d["kind"] = h.kind == BlobKind::AFEL ? "AFEL" : "AFBL";
            return d;
        },
        "path"_a, "The AFBL/AFEL JSON header as a dict, plus `kind`.");

    m.def(
        "combine_field",
        [](const std::vector<std::string> &paths, const std::vector<uint64_t> &steps,
           const std::vector<double> &factors, const std::string &out_path, const std::vector<nb::dict> &derive,
           uint64_t n_components, const std::string &name) {
            CombineOptions opts;
            for (const nb::dict &d : derive)
                opts.derive.push_back(spec_from_dict(d));
            opts.n_components = n_components;
            opts.name = name;
            const auto refs = stride_refs(paths, steps);
            const std::vector<float> f = to_f32(factors);
            FieldOpResult r;
            {
                nb::gil_scoped_release release;
                r = combine_field_files(refs, f, out_path, opts);
            }
            return json_loads(field_op_result_json(r));
        },
        "paths"_a, "steps"_a, "factors"_a, "out_path"_a, "derive"_a = std::vector<nb::dict>(), "n_components"_a = 0,
        "name"_a = "",
        "Materialise one load combination from baked strides and write a single-step AFBL/AFEL blob.\n"
        "Term t is step steps[t] of the blob at paths[t] (`steps` may be one step for every path).\n"
        "`derive` is a list of {'op', 'args', 'out'} applied in order after superposition -- in place,\n"
        "or into a new `n_components`-column layout (NaN-initialised) when n_components > 0. Returns\n"
        "the stats dict (per-step scalar ranges + timings). The same function the wasm module's\n"
        "combineField calls: identical bytes.");

    m.def(
        "envelope_field",
        [](const std::vector<std::string> &paths, const std::vector<uint64_t> &steps, const std::string &out_path,
           const std::string &gov_path) {
            const auto refs = stride_refs(paths, steps);
            FieldOpResult r;
            {
                nb::gil_scoped_release release;
                r = envelope_field_files(refs, out_path, gov_path);
            }
            return json_loads(field_op_result_json(r));
        },
        "paths"_a, "steps"_a, "out_path"_a, "gov_path"_a = "",
        "Envelope over cases: a two-step blob (0 = max, 1 = min) at out_path and, when gov_path is\n"
        "given, the governing case indices as an AFGV sidecar (16-byte header: 'AFGV', uint32 version 1,\n"
        "uint32 n_cases, uint32 0; then little-endian uint16 [2, rows, n_components]).");

    m.attr("DERIVE_OPS") =
        nb::make_tuple("copy", "plane_von_mises", "plane_principal", "plane_principal_1", "plane_principal_2",
                       "magnitude", "magnitude3", "shell_decompose", "shell_resultants");
    m.attr("BLOB_HEADER_BYTES") = BLOB_HEADER_BYTES;
}
