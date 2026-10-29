#include "sin_reader.h"

#include <algorithm>

namespace adacpp::fea::sin {

namespace {

constexpr int64_t SLOT_STRIDE = 8;

// The NFIELD word of a non-zero pointer lies in the file and is an integral float in [1, 1024].
bool valid_pointer(const Bytes &b, int64_t p) {
    const int64_t nfield_byte = (p - 1) * 4;
    if (nfield_byte < 0 || nfield_byte + 4 > static_cast<int64_t>(b.size))
        return false;
    const float v = b.f32(p - 1);
    return v >= 1.0f && v <= 1024.0f && v == std::trunc(v);
}

} // namespace

std::size_t first_invalid(const Bytes &b, const int64_t *ptrs, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i)
        if (ptrs[i] != 0 && !valid_pointer(b, ptrs[i]))
            return i;
    return n;
}

std::vector<int64_t> read_pointer_table(const Bytes &b, int64_t offset, int64_t max_slots) {
    const auto file_end = static_cast<int64_t>(b.size);
    const int64_t avail = offset <= file_end ? (file_end - offset) / SLOT_STRIDE : 0;
    max_slots = std::min(max_slots, avail);
    std::vector<int64_t> out;
    if (max_slots <= 0)
        return out;
    if (offset < 0)
        throw std::invalid_argument("offset must be non-negative and no greater than buffer length");
    for (int64_t i = 0; i < max_slots; ++i) {
        const int64_t p = b.u32(static_cast<std::size_t>(offset + i * SLOT_STRIDE + 4));
        if (p != 0 && !valid_pointer(b, p))
            break;
        out.push_back(p);
    }
    return out;
}

Ragged gather_ragged(const Bytes &b, const int64_t *ptrs, std::size_t n) {
    const auto file_end = static_cast<int64_t>(b.size);
    Ragged out;
    std::vector<int64_t> kept;
    for (std::size_t r = 0; r < n; ++r) {
        const int64_t p = ptrs[r];
        if (p < 1 || (p - 1) * 4 + 4 > file_end)
            continue;
        const int64_t nfield = np_int64(b.f32(p - 1));
        if (nfield == std::numeric_limits<int64_t>::min())
            continue;
        const int64_t n_data = nfield - 1;
        if (n_data <= 0 || p * 4 > file_end || n_data > (file_end - p * 4) / 4)
            continue;
        kept.push_back(p);
        out.n_data.push_back(n_data);
    }
    std::size_t total = 0;
    for (int64_t v : out.n_data)
        total += static_cast<std::size_t>(v);
    out.words.resize(total);
    float *dst = out.words.data();
    for (std::size_t r = 0; r < kept.size(); ++r) {
        const auto len = static_cast<std::size_t>(out.n_data[r]);
        std::memcpy(dst, b.data + static_cast<std::size_t>(kept[r]) * 4, len * 4);
        dst += len;
    }
    return out;
}

std::optional<RaggedRows> split_records(const int64_t *n_data, std::size_t n, const float *words, std::size_t n_words,
                                        std::size_t value_start) {
    if (value_start < 1)
        throw std::invalid_argument("split_records: value_start must be at least 1 (NFIELD)");
    const std::size_t n_head = value_start - 1;
    std::size_t total = 0;
    for (std::size_t r = 0; r < n; ++r) {
        if (n_data[r] + 1 < static_cast<int64_t>(value_start))
            return std::nullopt;
        total += static_cast<std::size_t>(n_data[r]);
    }
    if (total != n_words)
        throw std::invalid_argument("split_records: words do not add up to the records' counts");
    RaggedRows out;
    out.value_start = value_start;
    out.heads.resize(n * value_start);
    out.lengths.resize(n);
    out.values.resize(total - n * n_head);
    const float *src = words;
    float *dst = out.values.data();
    for (std::size_t r = 0; r < n; ++r) {
        double *head = out.heads.data() + r * value_start;
        head[0] = static_cast<double>(n_data[r] + 1);
        for (std::size_t j = 0; j < n_head; ++j)
            head[1 + j] = static_cast<double>(src[j]);
        const auto len = static_cast<std::size_t>(n_data[r]) - n_head;
        out.lengths[r] = static_cast<int64_t>(len);
        std::memcpy(dst, src + n_head, len * 4);
        src += n_data[r];
        dst += len;
    }
    return out;
}

RaggedRows present_complex(const double *heads, const int64_t *lengths, const float *values, std::size_t rows,
                           std::size_t n_values, std::size_t value_start, const uint8_t *is_complex, double phase) {
    const float c = static_cast<float>(1.0 * std::cos(phase));
    const float s = static_cast<float>(1.0 * std::sin(phase));
    RaggedRows out;
    out.value_start = value_start;
    out.heads.assign(heads, heads + rows * value_start);
    out.lengths.resize(rows);
    std::size_t total = 0;
    for (std::size_t r = 0; r < rows; ++r) {
        if (lengths[r] < 0 || (is_complex[r] && lengths[r] % 2))
            throw std::invalid_argument("present_complex: a flagged row has an odd count of values");
        out.lengths[r] = is_complex[r] ? lengths[r] / 2 : lengths[r];
        total += static_cast<std::size_t>(out.lengths[r]);
    }
    out.values.resize(total);
    const float *src = values;
    const float *end = values + n_values;
    float *dst = out.values.data();
    for (std::size_t r = 0; r < rows; ++r) {
        const auto len = static_cast<std::size_t>(lengths[r]);
        if (src + len > end)
            throw std::invalid_argument("present_complex: values shorter than the rows' lengths");
        if (!is_complex[r]) {
            std::memcpy(dst, src, len * 4);
            dst += len;
        } else {
            for (std::size_t j = 0; j < len; j += 2) {
                float v = src[j] * c;
                if (s != 0.0f)
                    v = v - src[j + 1] * s;
                *dst++ = v;
            }
            out.heads[r * value_start] = static_cast<double>(value_start + static_cast<std::size_t>(out.lengths[r]));
        }
        src += len;
    }
    return out;
}

CaseIndex CaseIndex::build(const Bytes &b, const int64_t *ptrs, std::size_t n) {
    const auto file_end = static_cast<int64_t>(b.size);
    CaseIndex idx;
    std::vector<int64_t> nz;
    for (std::size_t i = 0; i < n; ++i)
        if (ptrs[i] >= 1 && ptrs[i] <= file_end / 4)
            nz.push_back(ptrs[i]);
    if (nz.empty()) {
        idx.starts_.push_back(0);
        return idx;
    }
    const float nf0 = b.f32(nz[0] - 1);
    bool uniform = true;
    int64_t min_p = nz[0];
    for (int64_t p : nz) {
        uniform = uniform && b.f32(p - 1) == nf0;
        min_p = std::min(min_p, p);
    }
    if (uniform)
        idx.nfield = static_cast<double>(nf0);
    idx.min_pointer = min_p;

    // (first data word, table position) of every record whose first data word lies in the file.
    std::vector<std::pair<int64_t, int64_t>> keyed;
    keyed.reserve(nz.size());
    int64_t max_p = 0;
    for (int64_t p : nz)
        if (p * 4 + 4 <= file_end) {
            keyed.emplace_back(np_int64(b.f32(p)), p);
            max_p = std::max(max_p, p);
        }
    // Stable: a word's records stay in table order.
    std::stable_sort(keyed.begin(), keyed.end(), [](const auto &l, const auto &r) { return l.first < r.first; });
    const bool narrow = max_p <= static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
    if (narrow)
        idx.ptr32_.reserve(keyed.size());
    else
        idx.ptr64_.reserve(keyed.size());
    for (std::size_t i = 0; i < keyed.size(); ++i) {
        if (i == 0 || keyed[i].first != keyed[i - 1].first) {
            idx.keys_.push_back(keyed[i].first);
            idx.starts_.push_back(i);
        }
        if (narrow)
            idx.ptr32_.push_back(static_cast<uint32_t>(keyed[i].second));
        else
            idx.ptr64_.push_back(keyed[i].second);
    }
    idx.starts_.push_back(keyed.size());
    return idx;
}

std::vector<int64_t> CaseIndex::pointers(int64_t word) const {
    auto it = std::lower_bound(keys_.begin(), keys_.end(), word);
    if (it == keys_.end() || *it != word)
        return {};
    const auto k = static_cast<std::size_t>(it - keys_.begin());
    if (!ptr32_.empty())
        return std::vector<int64_t>(ptr32_.begin() + starts_[k], ptr32_.begin() + starts_[k + 1]);
    return std::vector<int64_t>(ptr64_.begin() + starts_[k], ptr64_.begin() + starts_[k + 1]);
}

std::vector<int64_t> CaseIndex::words() const {
    return keys_;
}

std::size_t CaseIndex::memory_bytes() const {
    return ptr32_.capacity() * 4 + ptr64_.capacity() * 8 + keys_.capacity() * 8 + starts_.capacity() * 8;
}

CaseTable gather_case(const Bytes &b, const CaseIndex &index, int64_t word) {
    const auto file_end = static_cast<int64_t>(b.size);
    CaseTable out;
    if (!index.min_pointer)
        return out;
    std::vector<int64_t> sel = index.pointers(word);
    int64_t nfield;
    if (!index.nfield) {
        if (sel.empty()) {
            out.kind = CaseTable::Ragged;
            return out;
        }
        const float nf0 = b.f32(sel[0] - 1);
        for (int64_t p : sel)
            if (!(b.f32(p - 1) == nf0)) {
                out.kind = CaseTable::Ragged;
                return out;
            }
        nfield = py_int(nf0);
    } else {
        nfield = py_int(static_cast<float>(*index.nfield));
    }
    const int64_t n_data = nfield - 1;
    if (n_data <= 0)
        return out;
    auto in_file = [&](int64_t p) { return p * 4 + n_data * 4 <= file_end; };
    bool any_in_file = false;
    if (index.nfield) {
        any_in_file = in_file(*index.min_pointer);
    } else {
        for (int64_t p : sel)
            if (in_file(p)) {
                any_in_file = true;
                break;
            }
    }
    if (!any_in_file)
        return out;
    sel.erase(std::remove_if(sel.begin(), sel.end(), [&](int64_t p) { return !in_file(p); }), sel.end());
    out.kind = CaseTable::Table;
    out.nfield = static_cast<std::size_t>(nfield);
    out.rows = sel.size();
    out.data.resize(out.rows * out.nfield);
    for (std::size_t r = 0; r < out.rows; ++r) {
        double *row = out.data.data() + r * out.nfield;
        row[0] = static_cast<double>(nfield);
        for (int64_t j = 0; j < n_data; ++j)
            row[1 + j] = static_cast<double>(b.f32(sel[r] + j));
    }
    return out;
}

FieldStats field_stats(const double *a, int64_t a_row, int64_t a_col, const double *b, int64_t b_row, int64_t b_col,
                       std::size_t rows, std::size_t cols) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FieldStats s{std::vector<double>(cols, nan), std::vector<double>(cols, nan), std::vector<double>(cols, nan),
                 std::vector<int64_t>(cols, 0)};
    // NaN-skipping running max of a magnitude: the first non-NaN value seeds it.
    auto fold = [](double &acc, double v) {
        if (!std::isnan(v) && (std::isnan(acc) || v > acc))
            acc = v;
    };
    for (std::size_t i = 0; i < rows; ++i) {
        const double *ra = a + static_cast<int64_t>(i) * a_row;
        const double *rb = b + static_cast<int64_t>(i) * b_row;
        for (std::size_t c = 0; c < cols; ++c) {
            const double va = ra[static_cast<int64_t>(c) * a_col];
            const double vb = rb[static_cast<int64_t>(c) * b_col];
            fold(s.max_abs_a[c], std::fabs(va));
            fold(s.max_abs_b[c], std::fabs(vb));
            const double vd = std::fabs(vb - va);
            if (!std::isnan(vd) && (std::isnan(s.max_abs_diff[c]) || vd > s.max_abs_diff[c])) {
                s.max_abs_diff[c] = vd;
                s.first_max_row[c] = static_cast<int64_t>(i);
            }
        }
    }
    return s;
}

} // namespace adacpp::fea::sin
