// Python bindings for the SIN reader loops (sin_reader.h), as adacpp.fea.sin.
//
// `buf` is the whole file as a 1-D uint8 array (np.frombuffer over the mmap: no copy). Nothing here keeps
// it: every call takes it again, so closing the mapping is never blocked by an object of this module.
// The GIL is released around every loop.
#include "fea_py_wrap.h"

#include <string>

#include <nanobind/stl/optional.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "shell_stress.h"
#include "sin_reader.h"

using namespace adacpp::fea::sin;

namespace {

using U8In = nb::ndarray<const uint8_t, nb::ndim<1>, nb::c_contig, nb::device::cpu>;
using I64In = nb::ndarray<const int64_t, nb::ndim<1>, nb::c_contig, nb::device::cpu>;
using F32In1 = nb::ndarray<const float, nb::ndim<1>, nb::c_contig, nb::device::cpu>;
using F64In2 = nb::ndarray<const double, nb::ndim<2>, nb::c_contig, nb::device::cpu>;
using F64Strided2 = nb::ndarray<const double, nb::ndim<2>, nb::device::cpu>;
using BoolIn = nb::ndarray<const bool, nb::ndim<1>, nb::c_contig, nb::device::cpu>;

Bytes bytes_of(const U8In &buf) {
    return Bytes{buf.data(), buf.shape(0)};
}

// Hand a vector's storage to numpy without a copy.
template <typename T> nb::ndarray<nb::numpy, T> to_numpy(std::vector<T> &&v, std::vector<std::size_t> shape = {}) {
    if (shape.empty())
        shape = {v.size()};
    auto *heap = new std::vector<T>(std::move(v));
    nb::capsule owner(heap, [](void *p) noexcept { delete static_cast<std::vector<T> *>(p); });
    return nb::ndarray<nb::numpy, T>(heap->data(), shape.size(), shape.data(), owner);
}

// (..., k) float32 or float64 in, (..., n_out) float64 out: shape and row count of a component-last array.
template <class T> using AnyIn = nb::ndarray<const T, nb::c_contig, nb::device::cpu>;

template <class T> std::pair<std::vector<std::size_t>, std::size_t> rows_of(const AnyIn<T> &a, std::size_t k) {
    if (a.ndim() < 1 || a.shape(a.ndim() - 1) != k)
        throw std::invalid_argument("expected the last axis to hold " + std::to_string(k) + " components");
    std::vector<std::size_t> shape(a.ndim());
    std::size_t rows = 1;
    for (std::size_t i = 0; i < a.ndim(); ++i) {
        shape[i] = a.shape(i);
        if (i + 1 < a.ndim())
            rows *= a.shape(i);
    }
    return {shape, rows};
}

template <class T> void def_shell_kernels(nb::module_ &m) {
    m.def(
        "shell_general_stress",
        [](const AnyIn<T> &basic) {
            auto [shape, rows] = rows_of(basic, 3);
            std::vector<double> out(rows * 4);
            {
                nb::gil_scoped_release nogil;
                adacpp::fea::shell::general_stress(basic.data(), rows, out.data());
            }
            shape.back() = 4;
            return to_numpy(std::move(out), shape);
        },
        "basic"_a);
    m.def(
        "shell_plane_principal",
        [](const AnyIn<T> &table, std::size_t cx, std::size_t cy, std::size_t ct) {
            if (table.ndim() < 1)
                throw std::invalid_argument("shell_plane_principal: need a component axis");
            const std::size_t ncols = table.shape(table.ndim() - 1);
            if (cx >= ncols || cy >= ncols || ct >= ncols)
                throw std::invalid_argument("shell_plane_principal: column out of range");
            auto [shape, rows] = rows_of(table, ncols);
            std::vector<double> out(rows * 2);
            {
                nb::gil_scoped_release nogil;
                adacpp::fea::shell::plane_principal(table.data(), rows, ncols, cx, cy, ct, out.data());
            }
            shape.back() = 2;
            return to_numpy(std::move(out), shape);
        },
        "table"_a, "cx"_a, "cy"_a, "ct"_a);
    m.def(
        "shell_decompose",
        [](const AnyIn<T> &bottom, const AnyIn<T> &top) {
            auto [shape, rows] = rows_of(bottom, 3);
            if (rows_of(top, 3).first != shape)
                throw std::invalid_argument("shell_decompose: bottom and top differ in shape");
            std::vector<double> out(rows * 7);
            {
                nb::gil_scoped_release nogil;
                adacpp::fea::shell::decompose_shell(bottom.data(), top.data(), rows, out.data());
            }
            shape.back() = 7;
            return to_numpy(std::move(out), shape);
        },
        "bottom"_a, "top"_a);
}

nb::tuple rows_tuple(RaggedRows &&r) {
    const std::size_t n = r.rows(), vs = r.value_start;
    return nb::make_tuple(to_numpy(std::move(r.heads), {n, vs}), to_numpy(std::move(r.lengths)),
                          to_numpy(std::move(r.values)));
}

} // namespace

void sin_module(nb::module_ &m) {
    m.doc() = "Per-record loops of a Sesam SIN result reader over the file's bytes (adapy's sin_reader.py). "
              "Each returns exactly what the numpy expression it replaces returns.";

    m.def(
        "first_invalid",
        [](const U8In &buf, const I64In &ptrs) {
            nb::gil_scoped_release nogil;
            return first_invalid(bytes_of(buf), ptrs.data(), ptrs.shape(0));
        },
        "buf"_a, "ptrs"_a,
        "Index of the first non-zero pointer whose NFIELD word is outside the file or not an integral float in "
        "[1, 1024]; len(ptrs) when there is none.");

    m.def(
        "read_pointer_table",
        [](const U8In &buf, int64_t offset, int64_t max_slots) {
            std::vector<int64_t> out;
            {
                nb::gil_scoped_release nogil;
                out = read_pointer_table(bytes_of(buf), offset, max_slots);
            }
            return to_numpy(std::move(out));
        },
        "buf"_a, "offset"_a, "max_slots"_a,
        "A block's pointer table (int64): the value word of each 8-byte slot from `offset`, at most `max_slots` "
        "and as far as the file holds, cut at the first invalid pointer.");

    m.def(
        "gather_ragged",
        [](const U8In &buf, const I64In &ptrs) {
            Ragged r;
            {
                nb::gil_scoped_release nogil;
                r = gather_ragged(bytes_of(buf), ptrs.data(), ptrs.shape(0));
            }
            return nb::make_tuple(to_numpy(std::move(r.n_data)), to_numpy(std::move(r.words)));
        },
        "buf"_a, "ptrs"_a,
        "(n_data int64, words float32) of the records at `ptrs` that iter_records would yield: each one's count "
        "of data words, and all their data words back to back.");

    m.def(
        "split_records",
        [](const I64In &n_data, const F32In1 &words, std::size_t value_start) -> nb::object {
            std::optional<RaggedRows> r;
            {
                nb::gil_scoped_release nogil;
                r = split_records(n_data.data(), n_data.shape(0), words.data(), words.shape(0), value_start);
            }
            if (!r)
                return nb::none();
            return rows_tuple(std::move(*r));
        },
        "n_data"_a, "words"_a, "value_start"_a,
        "(heads float64 [n, value_start], lengths int64, values float32) of records [NFIELD, *data]: NFIELD and "
        "the words before the values, then the values. None when a record is too short for its head words.");

    m.def(
        "present_complex",
        [](const F64In2 &heads, const I64In &lengths, const F32In1 &values, const BoolIn &is_complex, double phase) {
            const std::size_t rows = lengths.shape(0);
            if (heads.shape(0) != rows || is_complex.shape(0) != rows)
                throw std::invalid_argument("present_complex: heads, lengths and is_complex differ in rows");
            RaggedRows r;
            {
                nb::gil_scoped_release nogil;
                r = present_complex(heads.data(), lengths.data(), values.data(), rows, values.shape(0), heads.shape(1),
                                    reinterpret_cast<const uint8_t *>(is_complex.data()), phase);
            }
            return rows_tuple(std::move(r));
        },
        "heads"_a, "lengths"_a, "values"_a, "is_complex"_a, "phase"_a,
        "Each flagged row's (R, I) value pairs as one float32 word R*c - I*s (c, s = float32 cos/sin of `phase`; "
        "the I term only when s != 0), its head word 0 set to value_start + its new length. Returns (heads, "
        "lengths, values).");

    m.def(
        "records",
        [](const U8In &buf, const I64In &ptrs, std::size_t begin, std::size_t end,
           std::optional<int64_t> where_first_word, std::optional<std::vector<int64_t>> where_second_word) {
            end = std::min(end, ptrs.shape(0));
            begin = std::min(begin, end);
            const Bytes b = bytes_of(buf);
            std::vector<std::pair<int64_t, std::size_t>> found;
            {
                nb::gil_scoped_release nogil;
                for_each_record(b, ptrs.data(), begin, end, where_first_word,
                                where_second_word ? &*where_second_word : nullptr,
                                [&](int64_t word, std::size_t n) { found.emplace_back(word, n); });
            }
            nb::list out;
            for (const auto &[word, n] : found) {
                nb::object t = nb::steal(PyTuple_New(static_cast<Py_ssize_t>(n)));
                for (std::size_t j = 0; j < n; ++j)
                    PyTuple_SetItem(t.ptr(), static_cast<Py_ssize_t>(j),
                                    PyFloat_FromDouble(static_cast<double>(b.f32(word + static_cast<int64_t>(j)))));
                out.append(t);
            }
            return out;
        },
        "buf"_a, "ptrs"_a, "begin"_a, "end"_a, "where_first_word"_a = nb::none(), "where_second_word"_a = nb::none(),
        "iter_records' tuples (data words as floats) for ptrs[begin:end], with the same per-record checks, "
        "filters and errors.");

    m.def(
        "field_stats",
        [](const F64Strided2 &a, const F64Strided2 &b) {
            if (a.shape(0) != b.shape(0) || a.shape(1) != b.shape(1))
                throw std::invalid_argument("field_stats: a and b differ in shape");
            FieldStats s;
            {
                nb::gil_scoped_release nogil;
                s = field_stats(a.data(), a.stride(0), a.stride(1), b.data(), b.stride(0), b.stride(1), a.shape(0),
                                a.shape(1));
            }
            return nb::make_tuple(to_numpy(std::move(s.max_abs_a)), to_numpy(std::move(s.max_abs_b)),
                                  to_numpy(std::move(s.max_abs_diff)), to_numpy(std::move(s.first_max_row)));
        },
        "a"_a, "b"_a,
        "(max |a|, max |b|, max |b - a|, first row of the last) per column of two float64 tables of one shape, "
        "NaN skipped (NaN for a column without a value; row 0 then). Strided views are read in place.");

    // derived_values.py's shell kernels: float32 or float64 in (float32 first, so a float32 array is not
    // converted), float64 out, the same bits as its numpy expressions.
    def_shell_kernels<float>(m);
    def_shell_kernels<double>(m);
    m.def(
        "shell_resultants",
        [](const AnyIn<double> &d, const nb::ndarray<const double, nb::ndim<1>, nb::c_contig, nb::device::cpu> &t) {
            auto [shape, rows] = rows_of(d, 7);
            const std::size_t n_t = t.shape(0);
            if (n_t == 0 || rows % n_t)
                throw std::invalid_argument("shell_resultants: rows are not a whole number per thickness");
            std::vector<double> out(rows * 6);
            {
                nb::gil_scoped_release nogil;
                adacpp::fea::shell::stress_resultants(d.data(), rows, t.data(), rows / n_t, out.data());
            }
            shape.back() = 6;
            return to_numpy(std::move(out), shape);
        },
        "d"_a.noconvert(), "thickness"_a.noconvert(),
        "R-STRESS from D-STRESS (..., 7) and one thickness per leading-axis entry (rows evenly split).");

    nb::class_<CaseIndex>(m, "CaseIndex",
                          "A block's records grouped by first data word (the result case for RV tables), built in "
                          "one pass. Holds pointers only: 4 bytes a record.")
        .def(
            "__init__",
            [](CaseIndex *self, const U8In &buf, const I64In &ptrs) {
                nb::gil_scoped_release nogil;
                new (self) CaseIndex(CaseIndex::build(bytes_of(buf), ptrs.data(), ptrs.shape(0)));
            },
            "buf"_a, "pointer_table"_a)
        .def_ro("nfield", &CaseIndex::nfield, "The NFIELD every in-file record shares; None when they differ.")
        .def_ro("min_pointer", &CaseIndex::min_pointer, "The smallest in-file pointer; None when there is none.")
        .def_prop_ro("nbytes", &CaseIndex::memory_bytes)
        .def("words", [](const CaseIndex &self) { return to_numpy(self.words()); })
        .def(
            "pointers", [](const CaseIndex &self, int64_t word) { return to_numpy(self.pointers(word)); }, "word"_a,
            "The pointers (int64, table order) of the records whose first data word is `word`.")
        .def(
            "gather",
            [](const CaseIndex &self, const U8In &buf, int64_t word) -> nb::object {
                CaseTable t;
                {
                    nb::gil_scoped_release nogil;
                    t = gather_case(bytes_of(buf), self, word);
                }
                if (t.kind == CaseTable::Ragged)
                    return nb::none();
                if (t.kind == CaseTable::Empty)
                    return nb::cast(to_numpy(std::vector<double>{}, {0, 0}));
                return nb::cast(to_numpy(std::move(t.data), {t.rows, t.nfield}));
            },
            "buf"_a, "word"_a,
            "Case `word` as a (rows, nfield) float64 table with NFIELD in column 0; (0, 0) when the block has "
            "nothing to read; None when the case's records differ in width.");
}
