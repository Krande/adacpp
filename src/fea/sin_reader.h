// Hot loops of a Sesam SIN (Norsam) result-file reader, over the file's bytes in memory (an mmap).
//
// The block headers, the superelement directory and every format heuristic stay with the caller (adapy's
// sin_reader.py); these are the per-record loops it runs over pointer tables of up to tens of millions of
// slots. Each one reproduces the caller's numpy expression exactly -- the same records kept or skipped, the
// same words, the same float64 widening -- so either path gives the same arrays.
//
// Layout (all little-endian): a pointer is the 32-bit-word index of a record's first data word (so the
// 1-based index of the word before it, which holds NFIELD as a float32: the record's word count, itself
// included). A pointer of 0 is an unused slot.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace adacpp::fea::sin {

struct Bytes {
    const uint8_t *data = nullptr;
    std::size_t size = 0;

    float f32(int64_t word) const {
        float v;
        std::memcpy(&v, data + static_cast<std::size_t>(word) * 4, 4);
        return v;
    }
    uint32_t u32(std::size_t byte) const {
        uint32_t v;
        std::memcpy(&v, data + byte, 4);
        return v;
    }
};

// numpy's float32 -> int64 cast as x86 performs it: truncation, and INT64_MIN for NaN / out of range.
inline int64_t np_int64(float v) {
    if (!(v >= -9.2233720368547758e18f && v < 9.2233720368547758e18f))
        return std::numeric_limits<int64_t>::min();
    return static_cast<int64_t>(v);
}

// Python's int(float) of a float32 word: ValueError for NaN, OverflowError for inf. A finite value whose
// magnitude no record or file can reach is clamped (the callers only compare it against sizes or ids).
inline int64_t py_int(float v) {
    if (std::isnan(v))
        throw std::invalid_argument("cannot convert float NaN to integer");
    if (std::isinf(v))
        throw std::overflow_error("cannot convert float infinity to integer");
    constexpr float lim = 4.611686e18f; // 2^62
    if (v >= lim)
        return int64_t{1} << 62;
    if (v <= -lim)
        return -(int64_t{1} << 62);
    return static_cast<int64_t>(v);
}

// Index of the first non-zero pointer whose NFIELD word lies outside the file or is not an integral float
// in [1, 1024]; `n` when there is none (sin_reader._first_invalid).
std::size_t first_invalid(const Bytes &b, const int64_t *ptrs, std::size_t n);

// A block's pointer table: the value half of each 8-byte slot from `offset`, at most `max_slots` and no
// further than the file holds, cut at the first invalid pointer (sin_reader._read_pointer_table).
std::vector<int64_t> read_pointer_table(const Bytes &b, int64_t offset, int64_t max_slots);

// sin_reader.SinFile.gather_ragged_records' tail over the given pointers: the records iter_records would
// yield (zero, out-of-file and data-less ones skipped), as each one's data-word count and all data words
// back to back.
struct Ragged {
    std::vector<int64_t> n_data;
    std::vector<float> words;
};
Ragged gather_ragged(const Bytes &b, const int64_t *ptrs, std::size_t n);

// rv_table._RaggedRows.from_records: split records `[NFIELD, *data]` into `value_start` head words (NFIELD
// and the words before the values, as float64) and the value words. Empty optional when a record is too
// short to hold its head words.
struct RaggedRows {
    std::size_t value_start = 0;
    std::vector<double> heads; // rows x value_start
    std::vector<int64_t> lengths;
    std::vector<float> values;
    std::size_t rows() const {
        return lengths.size();
    }
};
std::optional<RaggedRows> split_records(const int64_t *n_data, std::size_t n, const float *words, std::size_t n_words,
                                        std::size_t value_start);

// read_sin._present_complex_ragged: each flagged row's (R, I) value pairs become one word, R*c - I*s in
// float32 with c = float32(cos(phase)), s = float32(sin(phase)) (the I term only when s != 0), and its head
// word 0 becomes value_start + the new length. Rows not flagged are copied. Flagged rows must have an even
// length (the caller checks and reports which case does not).
RaggedRows present_complex(const double *heads, const int64_t *lengths, const float *values, std::size_t rows,
                           std::size_t n_values, std::size_t value_start, const uint8_t *is_complex, double phase);

// A block's records grouped by first data word (sin_reader._CaseIndex): within a group in table order,
// groups by ascending word.
class CaseIndex {
public:
    static CaseIndex build(const Bytes &b, const int64_t *ptrs, std::size_t n);

    // The NFIELD every in-file record shares, none when they differ.
    std::optional<double> nfield;
    // The smallest in-file pointer, none when there is none.
    std::optional<int64_t> min_pointer;

    std::vector<int64_t> pointers(int64_t word) const;
    std::vector<int64_t> words() const;
    std::size_t memory_bytes() const;

private:
    // 32-bit pointers while they fit (files under 16 GiB), otherwise 64-bit.
    std::vector<uint32_t> ptr32_;
    std::vector<int64_t> ptr64_;
    std::vector<int64_t> keys_;
    std::vector<std::size_t> starts_; // keys_.size() + 1 entries
};

// sin_reader._gather_case: one case's fixed-width records as a (rows, nfield) float64 table, NFIELD in
// column 0. `kind` says which of the caller's results it is: no table (the case is ragged), the empty
// (0, 0) table, or rows x nfield (rows may be 0).
struct CaseTable {
    enum Kind { Ragged, Empty, Table } kind = Empty;
    std::size_t rows = 0, nfield = 0;
    std::vector<double> data;
};
CaseTable gather_case(const Bytes &b, const CaseIndex &index, int64_t word);

// iter_records' per-record loop over (pre-filtered) pointers [begin, end): calls `emit(first_word, n_data)`
// for every record it would yield, with the same checks in the same order and the same errors.
template <class Emit>
void for_each_record(const Bytes &b, const int64_t *ptrs, std::size_t begin, std::size_t end,
                     const std::optional<int64_t> &where_first_word, const std::vector<int64_t> *where_second_word,
                     Emit &&emit) {
    const auto file_end = static_cast<int64_t>(b.size);
    for (std::size_t r = begin; r < end; ++r) {
        const int64_t wp = ptrs[r];
        if (wp == 0)
            continue;
        const int64_t nfield_byte = (wp - 1) * 4;
        if (nfield_byte < 0 || nfield_byte + 4 > file_end)
            continue;
        const int64_t n_data = py_int(b.f32(wp - 1)) - 1;
        if (n_data <= 0)
            continue;
        const int64_t data_byte = wp * 4;
        if (data_byte > file_end || n_data > (file_end - data_byte) / 4)
            continue;
        if (where_first_word && py_int(b.f32(wp)) != *where_first_word)
            continue;
        if (where_second_word) {
            if (n_data < 2)
                continue;
            const int64_t second = py_int(b.f32(wp + 1));
            bool found = false;
            for (int64_t v : *where_second_word)
                if (v == second) {
                    found = true;
                    break;
                }
            if (!found)
                continue;
        }
        emit(wp, static_cast<std::size_t>(n_data));
    }
}

// compare._field_diff's statistics of two aligned (rows x cols) float64 tables, in one pass: per column the
// largest |a|, |b| and |b - a| ignoring NaN (NaN when a column has none), and the first row where |b - a|
// reaches its largest value (0 when there is none, as numpy's argmax over no hit gives). Strides are in
// elements.
struct FieldStats {
    std::vector<double> max_abs_a, max_abs_b, max_abs_diff;
    std::vector<int64_t> first_max_row;
};
FieldStats field_stats(const double *a, int64_t a_row, int64_t a_col, const double *b, int64_t b_row, int64_t b_col,
                       std::size_t rows, std::size_t cols);

} // namespace adacpp::fea::sin
