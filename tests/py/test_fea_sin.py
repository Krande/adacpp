"""adacpp.fea.sin against numpy references: the expressions adapy's SIN reader runs without adacpp.

Every buffer here is synthetic: records ``[NFIELD, *data]`` written at chosen word offsets, with pointer
tables (the word index of each record's first data word) pointing at them.
"""

import math

import numpy as np
import pytest

fea = pytest.importorskip("adacpp.fea")
sin = fea.sin


def assert_bits_equal(a, b):
    a, b = np.asarray(a), np.asarray(b)
    assert a.dtype == b.dtype and a.shape == b.shape, (a.dtype, b.dtype, a.shape, b.shape)
    assert a.tobytes() == b.tobytes()


def build_file(records, n_words=None, gap=3, rng=None):
    """Words with each record ``[nfield, *data]`` laid out after ``gap`` filler words; returns
    ``(uint8 buffer, pointers)``."""
    words, ptrs = [], []
    for rec in records:
        words += [7.5] * gap
        words.append(float(len(rec) + 1))
        ptrs.append(len(words))  # the first data word's index
        words += list(rec)
    if n_words is not None:
        words += [0.0] * (n_words - len(words))
    buf = np.asarray(words, dtype=np.float32).view(np.uint8).copy()
    return buf, np.asarray(ptrs, dtype=np.int64)


# --- numpy references (adapy's sin_reader.py / rv_table.py / read_sin.py / compare.py) -----------------


def ref_first_invalid(buf, ptrs):
    f32 = buf.view(np.float32) if buf.size % 4 == 0 else np.frombuffer(buf[: buf.size // 4 * 4], np.float32)
    file_end = buf.size
    nonzero = ptrs != 0
    nfield_bytes = (ptrs - 1) * 4
    in_bounds = (nfield_bytes >= 0) & (nfield_bytes + 4 <= file_end)
    word_idx = np.where(in_bounds & nonzero, nfield_bytes // 4, 0).astype(np.int64)
    at = f32[word_idx] if f32.size else np.zeros(ptrs.shape, np.float32)
    with np.errstate(invalid="ignore"):
        as_int = at.astype(np.int32).astype(np.float32)
    like = (at == as_int) & (at >= 1.0) & (at <= 1024.0)
    invalid = nonzero & ~(in_bounds & like)
    return int(np.argmax(invalid)) if invalid.any() else int(ptrs.size)


def ref_gather_ragged(buf, ptrs):
    f32 = buf.view(np.float32)
    file_end = buf.size
    ptrs = ptrs[(ptrs != 0) & (ptrs >= 1) & ((ptrs - 1) * 4 + 4 <= file_end)]
    n_data = f32[ptrs - 1].astype(np.int64) - 1 if ptrs.size else np.empty(0, dtype=np.int64)
    keep = (n_data > 0) & (ptrs * 4 + n_data * 4 <= file_end)
    ptrs, n_data = ptrs[keep], n_data[keep]
    total = int(n_data.sum())
    if not total:
        return n_data, np.empty(0, dtype=np.float32)
    offsets = np.cumsum(n_data) - n_data
    within = np.arange(total, dtype=np.int64) - np.repeat(offsets, n_data)
    return n_data, f32[np.repeat(ptrs, n_data) + within]


def ref_split(n_data, words, value_start):
    lengths = n_data + 1 - value_start
    if np.any(lengths < 0):
        return None
    n = len(n_data)
    n_head = value_start - 1
    offsets = np.cumsum(n_data) - n_data
    within = np.arange(int(n_data.sum()), dtype=np.int64) - np.repeat(offsets, n_data)
    heads = np.empty((n, value_start), dtype=np.float64)
    heads[:, 0] = n_data + 1
    heads[:, 1:] = words[within < n_head].reshape(n, n_head)
    return heads, lengths, np.asarray(words[within >= n_head], dtype=np.float32)


def ref_present(heads, lengths, values, is_complex, phase, value_start):
    c = np.float32(math.cos(phase))
    s = np.float32(math.sin(phase))
    offsets = np.cumsum(lengths) - lengths
    rows = np.flatnonzero(is_complex)
    pairs = lengths[rows] // 2
    before = np.cumsum(pairs) - pairs
    j = np.arange(int(pairs.sum()), dtype=np.int64) - np.repeat(before, pairs)
    real_pos = np.repeat(offsets[rows], pairs) + 2 * j
    imag_pos = real_pos + 1
    presented = values[real_pos] * c
    if s != 0:
        presented = presented - values[imag_pos] * s
    out = np.delete(values, imag_pos)
    out[real_pos - np.repeat(before, pairs) - j] = presented
    new_lengths = np.where(is_complex, lengths // 2, lengths)
    heads = heads.copy()
    heads[is_complex, 0] = value_start + new_lengths[is_complex]
    return heads, new_lengths, out


def ref_nanmax_abs(x):
    if x.shape[0] == 0:
        return np.full(x.shape[1], np.nan)
    with np.errstate(invalid="ignore"):
        out = np.abs(np.maximum(x.max(axis=0), -x.min(axis=0)))
        nan = np.isnan(out)
        if nan.any():
            out[nan] = np.fmax.reduce(np.abs(x[:, nan]), axis=0)
    return out


# --- pointer tables ------------------------------------------------------------------------------------


def test_first_invalid_edge_cases():
    buf, ptrs = build_file([[1, 2, 3], [4, 5], [6, 7, 8, 9]])
    f32 = buf.view(np.float32)
    cases = {
        "all valid": ptrs,
        "zeros are sparse slots": np.array([0, ptrs[0], 0, ptrs[1]], np.int64),
        "empty": np.empty(0, np.int64),
        "pointer 1 past the file": np.array([ptrs[0], f32.size + 2], np.int64),
        "NFIELD word not integral": np.array([ptrs[0], 2], np.int64),  # word 1 is filler 7.5
        "NFIELD word zero": np.array([ptrs[1], ptrs[1] + 1], np.int64),
    }
    for name, p in cases.items():
        assert sin.first_invalid(buf, p) == ref_first_invalid(buf, p), name


def test_first_invalid_nfield_range_and_nan():
    words = np.array([1.0, 1024.0, 1025.0, 0.5, np.nan, np.inf, -3.0, 2.0], dtype=np.float32)
    buf = words.view(np.uint8).copy()
    for k in range(len(words)):
        p = np.array([k + 1], np.int64)  # NFIELD at word k
        assert sin.first_invalid(buf, p) == ref_first_invalid(buf, p), words[k]


def test_read_pointer_table_cuts_at_first_invalid():
    buf, ptrs = build_file([[1, 2]] * 5)
    f32 = buf.view(np.float32)
    # A table of 8-byte slots (value in the high word) after the records: 5 real pointers, a zero, then junk.
    slots = np.zeros((8, 2), dtype=np.uint32)
    slots[:5, 1] = ptrs
    slots[6, 1] = 3  # word 2 is filler 7.5: invalid
    slots[7, 1] = ptrs[0]
    offset = buf.size
    whole = np.concatenate([buf, slots.ravel().view(np.uint8)])
    table = sin.read_pointer_table(whole, offset, 100)
    expected = slots[:, 1].astype(np.int64)
    assert_bits_equal(table, expected[: ref_first_invalid(whole, expected)])
    assert_bits_equal(sin.read_pointer_table(whole, offset, 3), expected[:3])
    assert sin.read_pointer_table(whole, whole.size + 64, 10).size == 0
    with pytest.raises(ValueError):
        sin.read_pointer_table(whole, -4, 10)
    assert f32.size  # the records part is untouched


# --- gathers -------------------------------------------------------------------------------------------


def test_gather_ragged_matches_numpy():
    rng = np.random.default_rng(3)
    records = [rng.normal(size=int(n)).astype(np.float32) for n in rng.integers(1, 12, size=300)]
    buf, ptrs = build_file(records)
    mixed = np.concatenate([ptrs, [0, buf.size // 4 + 5, 2]]).astype(np.int64)  # zero, out of file, filler
    rng.shuffle(mixed)
    n_data, words = sin.gather_ragged(buf, mixed)
    ref_n, ref_w = ref_gather_ragged(buf, mixed)
    assert_bits_equal(n_data, ref_n)
    assert_bits_equal(words, ref_w)


def test_split_records_matches_numpy_and_refuses_short_rows():
    rng = np.random.default_rng(4)
    n_data = rng.integers(4, 15, size=200).astype(np.int64)
    words = rng.normal(size=int(n_data.sum())).astype(np.float32)
    for value_start in (1, 3, 5):
        got = sin.split_records(n_data, words, value_start)
        ref = ref_split(n_data, words, value_start)
        for g, r in zip(got, ref):
            assert_bits_equal(g, r)
    assert sin.split_records(np.array([2, 8], np.int64), np.zeros(10, np.float32), 5) is None


@pytest.mark.parametrize("phase", [0.0, -math.pi / 2, 0.7, math.pi])
def test_present_complex_matches_numpy(phase):
    rng = np.random.default_rng(5)
    rows, value_start = 400, 4
    lengths = (2 * rng.integers(1, 8, size=rows)).astype(np.int64)
    values = rng.normal(size=int(lengths.sum())).astype(np.float32)
    heads = rng.normal(size=(rows, value_start))
    is_complex = rng.random(rows) < 0.4
    got = sin.present_complex(heads, lengths, values, is_complex, phase)
    ref = ref_present(heads, lengths, values, is_complex, phase, value_start)
    for g, r in zip(got, ref):
        assert_bits_equal(g, r)


def test_present_complex_refuses_odd_flagged_row():
    with pytest.raises(ValueError):
        sin.present_complex(np.zeros((1, 2)), np.array([3], np.int64), np.zeros(3, np.float32), np.array([True]), 0.0)


def test_case_index_groups_by_first_word_in_table_order():
    rng = np.random.default_rng(6)
    cases = rng.integers(1, 6, size=500)
    records = [[float(c), *rng.normal(size=5)] for c in cases]
    buf, ptrs = build_file(records)
    idx = sin.CaseIndex(buf, np.concatenate([[0], ptrs]).astype(np.int64))
    assert idx.nfield == 7.0 and idx.min_pointer == int(ptrs.min())
    assert_bits_equal(idx.words(), np.unique(cases).astype(np.int64))
    f32 = buf.view(np.float32)
    for c in np.unique(cases):
        want = ptrs[cases == c]
        assert_bits_equal(idx.pointers(int(c)), want)
        table = idx.gather(buf, int(c))
        ref = np.empty((want.size, 7))
        ref[:, 0] = 7.0
        ref[:, 1:] = f32[want[:, None] + np.arange(6)]
        assert_bits_equal(table, ref)
    assert idx.pointers(99).size == 0
    assert idx.gather(buf, 99).shape == (0, 7)


def test_case_index_ragged_and_empty():
    buf, ptrs = build_file([[1.0, 2.0], [1.0, 2.0, 3.0], [2.0, 5.0], [2.0, 6.0]])
    idx = sin.CaseIndex(buf, ptrs)
    assert idx.nfield is None
    assert idx.gather(buf, 1) is None  # case 1 mixes widths
    assert idx.gather(buf, 2).shape == (2, 3)
    assert idx.gather(buf, 7) is None  # no records and no common width
    empty = sin.CaseIndex(buf, np.zeros(4, np.int64))
    assert empty.min_pointer is None and empty.gather(buf, 1).shape == (0, 0)


def test_records_match_struct_unpack_with_filters():
    rng = np.random.default_rng(7)
    records = [[float(rng.integers(1, 4)), float(rng.integers(10, 14)), *rng.normal(size=3)] for _ in range(100)]
    buf, ptrs = build_file(records)
    f32 = buf.view(np.float32)

    def ref(first=None, second=None):
        out = []
        for p in ptrs:
            n = int(f32[p - 1]) - 1
            row = tuple(float(x) for x in f32[p : p + n])
            if first is not None and int(row[0]) != first:
                continue
            if second is not None and int(row[1]) not in second:
                continue
            out.append(row)
        return out

    assert sin.records(buf, ptrs, 0, len(ptrs)) == ref()
    assert sin.records(buf, ptrs, 0, len(ptrs), 2) == ref(2)
    assert sin.records(buf, ptrs, 0, len(ptrs), 2, [11, 13]) == ref(2, {11, 13})
    assert sin.records(buf, ptrs, 10, 20) == ref()[10:20]


def test_records_nan_nfield_raises_like_int():
    buf = np.array([np.nan, 1.0, 2.0], np.float32).view(np.uint8).copy()
    with pytest.raises(ValueError):
        sin.records(buf, np.array([1], np.int64), 0, 1)  # NFIELD (word 0) is NaN


# --- comparison statistics -----------------------------------------------------------------------------


def test_field_stats_match_numpy_with_nan_and_strides():
    rng = np.random.default_rng(8)
    full_a = rng.normal(size=(5000, 11))
    full_b = full_a + rng.normal(scale=1e-3, size=full_a.shape)
    full_a[rng.random(full_a.shape) < 0.01] = np.nan
    full_a[:, 5] = np.nan  # an all-NaN column
    full_b[3, 4] = -0.0
    full_a[3, 4] = 0.0
    a, b = full_a[:, 2:], full_b[:, 2:]  # strided views, as compare passes them
    ma, mb, md, first = sin.field_stats(a, b)
    d = b - a
    ref_md = ref_nanmax_abs(d)
    assert_bits_equal(ma, ref_nanmax_abs(a))
    assert_bits_equal(mb, ref_nanmax_abs(b))
    assert_bits_equal(md, ref_md)
    ref_first = np.argmax((d == ref_md) | (d == -ref_md), axis=0)
    assert_bits_equal(first, ref_first.astype(np.int64))


def test_field_stats_first_row_of_a_tie():
    a = np.zeros((4, 1))
    b = np.array([[1.0], [-2.0], [2.0], [-2.0]])
    assert sin.field_stats(a, b)[3].tolist() == [1]


# --- derived shell stresses (adapy's derived_values.py) ------------------------------------------------


def ref_von_mises(x, y, t):
    return np.sqrt(x * x + y * y - x * y + 3.0 * t * t)


def ref_principal(x, y, t):
    centre = 0.5 * (x + y)
    radius = np.sqrt((0.5 * (x - y)) ** 2 + t * t)
    return np.stack((centre + radius, centre - radius), axis=-1)


def ref_decompose(bottom, top):
    bottom, top = np.asarray(bottom, dtype=float), np.asarray(top, dtype=float)
    m, b = 0.5 * (top + bottom), 0.5 * (top - bottom)
    mv = ref_von_mises(m[..., 0], m[..., 1], m[..., 2])
    return np.stack((m[..., 0], m[..., 1], b[..., 0], b[..., 1], m[..., 2], b[..., 2], mv), axis=-1)


def ref_resultants(d, t):
    t2 = t * t / 6.0
    return np.stack((d[..., 0] * t, d[..., 4] * t, d[..., 1] * t, d[..., 5] * t2, d[..., 2] * t2, d[..., 3] * t2), -1)


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_shell_kernels_match_numpy(dtype):
    rng = np.random.default_rng(9)
    bottom = (rng.normal(size=(300, 8, 3)) * 1e8).astype(dtype)
    top = (rng.normal(size=(300, 8, 3)) * 1e8).astype(dtype)
    wide = bottom.astype(float)
    vm = ref_von_mises(wide[..., 0], wide[..., 1], wide[..., 2])
    assert_bits_equal(sin.shell_general_stress(bottom), np.concatenate((wide, vm[..., None]), axis=-1))
    assert_bits_equal(
        sin.shell_plane_principal(bottom, 0, 1, 2), ref_principal(wide[..., 0], wide[..., 1], wide[..., 2])
    )
    d = sin.shell_decompose(bottom, top)
    assert_bits_equal(d, ref_decompose(bottom, top))
    assert_bits_equal(sin.shell_plane_principal(d, 0, 1, 4), ref_principal(d[..., 0], d[..., 1], d[..., 4]))
    t = rng.uniform(0.005, 0.05, size=300)
    per_row = np.ascontiguousarray(np.broadcast_to(t[:, None], d.shape[:-1])).ravel()
    assert_bits_equal(sin.shell_resultants(d, per_row), ref_resultants(d, t[:, None]))
    assert_bits_equal(sin.shell_resultants(d, t), ref_resultants(d, t[:, None]))
